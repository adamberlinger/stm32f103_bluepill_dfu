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
#include "usb_dfu.h"

const uint8_t get_command_response[4] = {0x00, 0x21, 0x41, 0x92};
uint8_t dfuse_command[6];
volatile dfu_cmd_status_t dfuse_cmd_status;
volatile uint32_t dfu_erase_address;
volatile dfu_status_t dfu_status;
volatile dfu_mode_t dfu_mode;

volatile uint32_t address_pointer;

static void dfu_process_status(void);
static void dfu_read(uint32_t address, uint32_t size);

volatile uint16_t write_buffer[32];
volatile uint32_t read_buffer[16];
volatile uint32_t write_size;
volatile uint32_t write_offset;

static void dfu_read(uint32_t address, uint32_t size){
    for(int offset = 0;offset < size;offset += 4){
      uint32_t rp = address + offset;
      if(rp == DFU_APP_RESET_HANDLER){
        read_buffer[offset >> 2] = 0;
      }
      else if(rp == 0x08000004){
        read_buffer[offset >> 2] = *((uint32_t*)DFU_APP_RESET_HANDLER);
      }
      else {
        read_buffer[offset >> 2] = *((uint32_t*)rp);
      }
    }


    usb_core_transmit0((void*)read_buffer, size);
}

void usb_dfu_request(const usb_request_t *request, const uint8_t *request_data){
    if(request->request_id == DFU_UPLOAD){
      if(request->value > 1){
        uint32_t addr = (request->value - 2) * 64 + address_pointer;
        /* TODO: check valid address ranged */
        dfu_read(addr, request->length);
        dfu_mode = DFU_MODE_UPLOAD;
      }
      else if(request->value == 0) {
        usb_core_transmit0(get_command_response, 4);
      }
      else {
        usb_core_status0(0);
      }
    }
    else if(request->request_id == DFU_DNLOAD){
      if(request->value > 0){
        write_size = (request->length > 64)?64:request->length;
        mem_copy16to8(request_data, (uint16_t*)write_buffer, write_size);

        dfuse_cmd_status = DFU_CMD_WRITE_PARSE;
        dfu_mode = DFU_MODE_DOWNLOAD;
        dfu_status.state = dfuDNBUSY;
        write_offset = (request->value - 2) * 64;
        usb_core_status0(1);

        GPIOC->BRR = (1 << 13);
      }
      else if(request->value == 0) {
        int size = (request->length > 5)?5:request->length;
        mem_copy16to8(request_data, dfuse_command, size);
        dfuse_command[5] = size;
        dfuse_cmd_status = DFU_CMD_PARSE;

        usb_core_status0(1);
      }
      else {
        usb_core_status0(0);
      }
    }
    else if(request->request_id == DFU_GETSTATUS){

      if(dfuse_cmd_status == DFU_CMD_WRITE_PARSE){
        dfuse_cmd_status = DFU_CMD_WRITE;
        usb_nvic_deinit();
      }
      else {
        dfu_process_status();
        usb_core_transmit0((void*)&dfu_status, 6);
      }
    }
    else if(request->request_id == DFU_CLRSTATUS){
      dfu_status.status = 0;
      dfu_status.state =dfuIDLE;
      dfu_mode = DFU_MODE_NONE;
      usb_core_status0(1);
    }
    else if(request->request_id == DFU_GETSTATE){
      usb_core_transmit0((void*)&dfu_status.state, 1);
    }
    else if(request->request_id == DFU_ABORT){
      dfu_status.status = 0;
      dfu_status.state =dfuIDLE;
      dfu_mode = DFU_MODE_NONE;
      usb_core_status0(1);
    }
    else {
      usb_core_status0(0);
    }
}

static void dfu_process_status(void){
  if(dfuse_cmd_status == DFU_CMD_PARSE){
    dfu_mode = DFU_MODE_DOWNLOAD;
    dfuse_cmd_status = DFU_CMD_IDLE;
    dfu_status.state = dfuDNBUSY;
    /* Set address pointer */
    if(dfuse_command[0] == 0x21){
      address_pointer = dfuse_command[1] | (dfuse_command[2] << 8) |
          (dfuse_command[3] << 16) | (dfuse_command[4] << 24);
      if(address_pointer >= 0x0800E000 || address_pointer < 0x08000000){
        dfu_status.state = dfuERROR;
        dfu_status.status = errTARGET;
      }
    }
    /* Erase command */
    else if(dfuse_command[0] == 0x41){
      if(dfuse_command[5] == 1){
        dfu_erase_address = 0xFFFFFFFF;
        dfuse_cmd_status = DFU_CMD_ERASE;
        GPIOC->BRR = (1 << 13);
      }
      else {
        dfu_erase_address = dfuse_command[1] | (dfuse_command[2] << 8) |
            (dfuse_command[3] << 16) | (dfuse_command[4] << 24);
        /* TODO: check DFU erase */
        dfuse_cmd_status = DFU_CMD_ERASE;
      }
    }
  }
  else if(dfuse_cmd_status == DFU_CMD_IDLE){
    if(dfu_status.state != dfuERROR) {
      dfu_status.state = dfu_mode;
      dfu_status.status = 0;
    }
  }
}

static void dfu_write_word(uint32_t addr, uint32_t value){
  volatile uint16_t* flash = (uint16_t*)addr;

  FLASH->CR |= FLASH_CR_PG;

  flash[0] = value & 0xFFFF;
  while(FLASH->SR & FLASH_SR_BSY);

  flash[1] = (value >> 16) & 0xFFFF;
  while(FLASH->SR & FLASH_SR_BSY);

  FLASH->CR &= ~(uint32_t)FLASH_CR_PG;
}

void reset_handler(void);

static void dfu_erase_sector(uint32_t addr){

  if((addr & 0xFFFE03FF) != 0x08000000){
    /* Invalid sector address */
    return;
  }
  else if((addr & 0x1FC00) >= (124 << 10)){
    /* Sector dedicated to bootloader */
    return;
  }

  FLASH->CR |= FLASH_CR_PER;
  FLASH->AR = addr;
  FLASH->CR |= FLASH_CR_STRT;

  while(FLASH->SR & FLASH_SR_BSY);

  FLASH->CR &= ~(uint32_t)FLASH_CR_PER;

  if(addr == 0x08000000){
      dfu_write_word(0x08000004, (uint32_t)&reset_handler);
      return;
  }
}

void usb_dfu_main_thread(){
  if(dfuse_cmd_status == DFU_CMD_ERASE){
    if(dfu_erase_address != 0xFFFFFFFF){
      dfu_erase_sector(dfu_erase_address);
    }
    else {
      uint32_t addr = 0x08000000;
      while(addr < 0x0800E000){
        dfu_erase_sector(addr);
        addr += 0x400;
      }
    }

    dfuse_cmd_status = DFU_CMD_IDLE;
    GPIOC->BSRR = (1 << 13);
  }
  else if(dfuse_cmd_status == DFU_CMD_WRITE) {
    uint32_t addr = address_pointer + write_offset;
    int i = 0;

    while((i*2) < write_size){
      uint32_t data = write_buffer[i] | (write_buffer[i+1] << 16);
      if(addr == 0x08000004){
        dfu_write_word(DFU_APP_RESET_HANDLER, data);
      }
      else if(addr != DFU_APP_RESET_HANDLER){
        dfu_write_word(addr,data);
      }
      addr += 4;
      i+=2;
    }

    dfuse_cmd_status = DFU_CMD_IDLE;
    GPIOC->BSRR = (1 << 13);

    usb_core_transmit0((void*)&dfu_status, 6);
    usb_nvic_init();
  }
}

void usb_dfu_enable(void){
  address_pointer = 0x08000000;

  /* Default to same value as F042 bootloader */
  dfu_status.status = 0xA;
  dfu_status.poll_timeout[0] = 0;
  dfu_status.poll_timeout[1] = 0;
  dfu_status.poll_timeout[2] = 0;
  dfu_status.state = 0xA;
  dfu_status.string = 0;

  dfu_mode = DFU_MODE_NONE;
  dfuse_cmd_status = DFU_CMD_IDLE;
}

void usb_dfu_data_received(uint16_t* data, uint16_t length){

}

uint8_t usb_dfu_rx_ready(void){
  return 1;
}
