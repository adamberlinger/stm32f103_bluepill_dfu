/*
 * BSD 3-Clause License
 * 
 * Copyright (c) 2022, Adam Berlinger
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "usb_core.h"
#include "usb_bsp.h"
#include "usb_desc.h"
#include "usb_dfu.h"
#include <string.h>

static volatile buffer_table_row_t *usb_buffer_table
    = (buffer_table_row_t*)USB_SRAM_START_ADDRESS;
static usb_state_t usb_state;

#define DESC_SIZE(requested_size,size)  ((requested_size < size) ? (requested_size) : (size))

static usb_request_t tmp_request;

static uint8_t str_buffer[128];

int usb_core_init(void){
  uint32_t i;
  usb_bsp_init();

  USB->CNTR &= ~USB_CNTR_PDWN;
  usb_ms_delay(1);
  USB->CNTR &= ~USB_CNTR_FRES;
  USB->ISTR = 0;

  usb_state.ms_ticks = 0;
  usb_state.address = 0;
  usb_state.address_pending = 0;

  for(i = 0;i < USB_NUM_TRANSMIT_BUFFERS;++i){
    usb_state.transfers[i].size = 0;
    usb_state.transfers[i].transfered = 0;
  }

  USB->BTABLE = 0;

  /* TODO: setup table */
  usb_buffer_table[0].addr_tx = sizeof(buffer_table_row_t) * USB_NUM_BUFFERS;
  usb_buffer_table[0].count_tx = 0; /* Nothing to transfer yet */
  usb_buffer_table[0].addr_rx = sizeof(buffer_table_row_t) * USB_NUM_BUFFERS + 64;
  usb_buffer_table[0].count_rx = 0x8400; /* 64-byte size */

  usb_buffer_table[1].addr_tx = sizeof(buffer_table_row_t) * USB_NUM_BUFFERS + 128;
  usb_buffer_table[1].count_tx = 0; /* Nothing to transfer yet */
  usb_buffer_table[1].addr_rx = 0;
  usb_buffer_table[1].count_rx = 0;

  usb_buffer_table[2].addr_tx = 0;
  usb_buffer_table[2].count_tx = 0;
  usb_buffer_table[2].addr_rx = sizeof(buffer_table_row_t) * USB_NUM_BUFFERS + 192;
  usb_buffer_table[2].count_rx = 0x8400; /* Nothing to transfer yet */

  /* Enable Start-of-frame and reset interrupts */
  USB->CNTR |= USB_CNTR_SOFM | USB_CNTR_RESETM;

  /* No internall pull-up on STM32F1 devices */
#ifndef STM32F1XX
  /* Enable internall pull-up on D+ line */
  USB->BCDR |= USB_BCDR_DPPU;
#endif

  return 0;
}

void usb_core_reset(){
  uint32_t i;

  /* Clear STAT_RX and STAT_TX */
  USB->EP0R &= 0x3030;
  USB->EP1R &= 0x3030;
  USB->EP2R &= 0x3030;
  USB->EP3R &= 0x3030;
  USB->EP4R &= 0x3030;
  USB->EP5R &= 0x3030;
  USB->EP6R &= 0x3030;
  USB->EP7R &= 0x3030;

  /* Enable EP0 RX */
  USB->EP0R = 0x3220;

  /* Clear device address */
  USB->DADDR = USB_DADDR_EF;

  usb_state.address = 0;
  usb_state.address_pending = 0;

  for(i = 0; i < USB_NUM_TRANSMIT_BUFFERS;++i){
    usb_state.transfers[i].size = 0;
  }
}

void mem_copy16_tousb(const void* source, void* dest, uint16_t size){
  size = (size+1) >> 1;
  const uint16_t* s = (const uint16_t*)source;
  usb_sram_t* d = (usb_sram_t*)dest;
  uint16_t i = 0;
  for(;i < size;++i){
    d[i] = s[i];
  }
}

void mem_copy16_fromusb(const void* source, void* dest, uint16_t size){
  size = (size+1) >> 1;
  const usb_sram_t* s = (const usb_sram_t*)source;
  uint16_t* d = (uint16_t*)dest;
  uint16_t i = 0;
  for(;i < size;++i){
    d[i] = s[i];
  }
}

void mem_copy16to8(const void* source, void* dest, uint16_t size){

  usb_sram_t *s = (usb_sram_t*)((uint32_t)source & 0xfffffffe);
  uint8_t *d = (uint8_t*) dest;
  uint32_t i;

  if((uint32_t)source & 0x1){
    d[0] = (s[0] >> 8) & 0xff;
    size--;
    if(size == 0) return;
    d++;
    s++;
  }


  for(i = 0;i < (size-1);i+=2){
    uint16_t s16 = s[i >> 1];
    d[i] = s16 & 0xFF;
    d[i+1] = (s16 >> 8) & 0xFF;
  }

  if(i == (size-1)){
    d[i] = s[i >> 1] & 0xFF;
  }
}

void mem_copy8to16(const void* source, void* dest, uint16_t size){
  uint8_t *s = (uint8_t*) source;
  usb_sram_t *d = (usb_sram_t*) dest;

  uint32_t i;
  for(i = 0;i < (size-1);i+=2){
    d[i >> 1] = s[i] | (s[i+1] << 8);
  }

  if(i == (size-1)){
    d[i >> 1] = s[i];
  }
}

void usb_core_status0(int success){
  if(success){
    usb_buffer_table[0].count_tx = 0;
    if(USB_EPx(0) & 0x20){
      USB_EPx(0) = 0x8290;
    }
    else {
      USB_EPx(0) = 0x82B0;
    }
  }
  else {
    if(USB_EPx(0) & 0x20){
      USB_EPx(0) = 0x82B0;
    }
    else {
      USB_EPx(0) = 0x8290;
    }
  }
}

void usb_core_transmit0(const uint8_t* data,uint16_t size){
  if(size > 64){
    mem_copy16_tousb(data, (uint16_t*)(usb_buffer_table[0].addr_tx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), 64);
    usb_buffer_table[0].count_tx = 64;
    usb_state.transfers[0].data = data + 64;
    usb_state.transfers[0].size = size - 64;
    usb_state.transfers[0].transfered = 0;
  }
  else {
    mem_copy16_tousb(data, (uint16_t*)(usb_buffer_table[0].addr_tx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), size);
    usb_buffer_table[0].count_tx = size;
  }
  if(USB_EPx(0) & 0x20){
    USB_EPx(0) = 0x8290;
  }
  else {
    USB_EPx(0) = 0x82B0;
  }
}

void usb_core_trasnsmit_string(const char* string, uint16_t max_size){
  uint16_t size = strlen(string);
  size = size*2+2;
  str_buffer[0] = size;
  str_buffer[1] = 0x03;
  int i = 2;
  while(*string != '\0' && i <= 126){
    str_buffer[i] = *string;
    str_buffer[i+1] = 0;
    i+=2;
    string++;
  }

  usb_core_transmit0((uint8_t*)str_buffer,DESC_SIZE(max_size,size));
}

void usb_core_transmit1(const uint8_t* data,uint16_t size){
  if(size > 64){
    mem_copy8to16(data, (uint16_t*)(usb_buffer_table[1].addr_tx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), 64);
    usb_buffer_table[1].count_tx = 64;
    usb_state.transfers[1].data = data + 64;
    usb_state.transfers[1].size = size - 64;
    usb_state.transfers[1].transfered = 0;
  }
  else {
    mem_copy8to16(data, (uint16_t*)(usb_buffer_table[1].addr_tx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), size);
    usb_buffer_table[1].count_tx = size;
    usb_state.transfers[1].size = 0;
    if(size == 64){
      usb_state.transfers[1].transfered = 2;
    }
    else {
      usb_state.transfers[1].transfered = 1;
    }
  }
  if(USB_EPx(1) & 0x20){
    USB_EPx(1) = 0x8091;
  }
  else {
    USB_EPx(1) = 0x80B1;
  }
}

uint8_t usb_core_is_ep1_free(){
  return (usb_state.transfers[1].size == 0 && usb_state.transfers[1].transfered == 0);
}

void usb_core_handle_request(const usb_request_t *request, const uint8_t* request_data){
  /* Class specific requests */
  if((request->request_type & 0x60) == 0x20){
    usb_dfu_request(request,request_data);
  }
  /* Standard requests */
  else if((request->request_type & 0x60) == 0x0){
    if(request->request_id == 0x6 ){
      if((request->value & 0xFF00) == 0x100){
        usb_core_transmit0((uint8_t*)device_descriptor,DESC_SIZE(request->length,DEVICE_DESCRIPTOR_SIZE));
      }
      else if((request->value & 0xFF00) == 0x200) {
        usb_core_transmit0((uint8_t*)config_descriptor,DESC_SIZE(request->length,CONFIG_DESCRIPTOR_SIZE));
      }
      /* Device qualifier descriptor */
      else if((request->value & 0xFF00) == 0x600) {
        usb_core_status0(0);
      }
      /* Get string descriptor */
      else if((request->value & 0xFF00) == 0x300){
        int index = request->value & 0xFF;
        if(index == 0){
          usb_core_transmit0((uint8_t*)string_descriptor0,DESC_SIZE(request->length,0x04));
        }
        else if(index <= STRING_DESCRIPTOR_COUNT){
          usb_core_trasnsmit_string(string_descriptors[index-1], request->length);
        }
        else {
          usb_core_status0(0);
        }
      }
      else {
        usb_core_status0(0);
      }
    }
    /* Set configuration */
    else if(request->request_id == 0x9){
      /* Enable DFU endpoints */
#if 0
      USB_EPx(2) = 0x3022;
      USB_EPx(1) = 0x8021;
      USB_EPx(3) = 0x8623;
#endif

      usb_dfu_enable();
      usb_core_status0(1);
    }
    /* Set address */
    else if(request->request_id == 0x5){
      usb_state.address = request->value;
      usb_state.address_pending = 1;
      usb_core_status0(1);
    }
  }
}

void usb_core_ep_tx(uint32_t ep){
  if(ep == 0){
    /* Clear TX flag */
    USB_EPx(0) = 0x8200;

    if(usb_state.address_pending){
      usb_state.address_pending = 0;
      USB->DADDR = USB_DADDR_EF | usb_state.address;
    }
  }
  else if(ep == 1){
    /* Clear TX flag */
    USB_EPx(1) = 0x8001;
  }
  if(usb_state.transfers[ep].size > 0){
    if(usb_state.transfers[ep].size > 64){
      mem_copy8to16(usb_state.transfers[ep].data,
           (uint16_t*)(usb_buffer_table[ep].addr_tx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), 64);
      usb_buffer_table[ep].count_tx = 64;
      usb_state.transfers[ep].data += 64;
      usb_state.transfers[ep].transfered += 64;
      usb_state.transfers[ep].size -= 64;
    }
    else {
      mem_copy8to16(usb_state.transfers[ep].data,
           (uint16_t*)(usb_buffer_table[ep].addr_tx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), usb_state.transfers[ep].size);
      usb_buffer_table[ep].count_tx = usb_state.transfers[ep].size;
      if(usb_state.transfers[ep].size == 64 && ep == 1){
        usb_state.transfers[ep].transfered = 2;
      }
      else{
        usb_state.transfers[ep].transfered = 1;
      }
      usb_state.transfers[ep].size = 0;
    }
    if(ep == 0){
      if(USB_EPx(0) & 0x20){
        USB_EPx(0) = 0x8290;
      }
      else {
        USB_EPx(0) = 0x82B0;
      }
    }
    else if(ep == 1){
      if(USB_EPx(1) & 0x20){
        USB_EPx(1) = 0x8091;
      }
      else {
        USB_EPx(1) = 0x80B1;
      }
    }
  }
  else {
    /* Send zero-length packet */
    if(usb_state.transfers[ep].transfered == 2 && ep == 1){
      usb_buffer_table[ep].count_tx = 0;
      if(USB_EPx(1) & 0x20){
        USB_EPx(1) = 0x8091;
      }
      else {
        USB_EPx(1) = 0x80B1;
      }
      usb_state.transfers[ep].transfered = 1;
    }
    else {
      usb_state.transfers[ep].transfered = 0;
    }
  }
}

void usb_core_ep_rx(uint32_t ep){
  uint16_t rx_size = usb_buffer_table[ep].count_rx & 0x03FF;
  if(ep == 0){
    /* Clear RX flag */
    USB_EPx(0) = 0x0280;

    if(USB->EP0R & USB_EP_SETUP){
      //const usb_request_t* rq = (const usb_request_t*)(usb_buffer_table[0].addr_rx + USB_SRAM_START_ADDRESS);
      mem_copy16_fromusb((void*)(usb_buffer_table[0].addr_rx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS),&tmp_request,8);
      if((tmp_request.request_type & 0x80) == 0x00 && tmp_request.length > 0){
        mem_copy16_fromusb((void*)(usb_buffer_table[0].addr_rx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS), &usb_state.request, 8);
        usb_state.control_state = USB_CONTROL_DATA_RX;
      }
      else {
        usb_core_handle_request(&tmp_request,NULL);
      }
    }
    else if(usb_state.control_state == USB_CONTROL_DATA_RX) {
      usb_core_handle_request(&usb_state.request,(const uint8_t*)(usb_buffer_table[0].addr_rx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS));
      usb_state.control_state = USB_CONTROL_IDLE;
    }

    /* Re-enable reception */
    USB_EPx(0) = 0x9280;
  }
  else if(ep == 2){
    uint16_t* rx_data = (uint16_t*)(usb_buffer_table[2].addr_rx * USB_SRAM_MULT + USB_SRAM_START_ADDRESS);

    /* Clear RX flag */
    USB_EPx(2) = 0x0082;
    usb_dfu_data_received(rx_data, rx_size);

    /* Re-enable reception */
    if(usb_dfu_rx_ready()){
      USB_EPx(2) = 0x9082;
    }
  }
}

void usb_interrupt(){
  uint32_t istr = USB->ISTR;
  if(istr & USB_ISTR_RESET){
    usb_core_reset();
    USB->ISTR = 0;
  }
  else {
    if(istr & USB_ISTR_SOF){
      usb_state.ms_ticks++;
      USB->ISTR = ~((uint16_t)USB_ISTR_SOF);

      /* Re-enable CDC reception when buffer free */
      if(((USB_EPx(2) & 0x3000) != 0x3000) && usb_dfu_rx_ready()){
        USB_EPx(2) = 0x9082;
      }
    }

    if(istr & USB_ISTR_CTR){
      if(istr & USB_ISTR_DIR){
        usb_core_ep_rx(istr & 0xF);
      }
      else {
        usb_core_ep_tx(istr & 0xF);
      }
    }
  }
}
