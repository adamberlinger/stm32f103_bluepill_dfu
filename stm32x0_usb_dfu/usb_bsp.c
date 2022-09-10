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
#include "usb_bsp.h"

int usb_clk_init(void){
#ifdef STM32F1XX
  RCC->APB1ENR |= RCC_APB1ENR_USBEN;
#else
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB1ENR |= RCC_APB1ENR_CRSEN | RCC_APB1ENR_USBEN;

#ifndef STM32F0
  if(!(SYSCFG->CFGR3 & (0x1 << 26))){
    uint32_t tries = 0x2000000;
    SYSCFG->CFGR3 |= SYSCFG_CFGR3_ENREF_HSI48 | 0x1;
    while(!(SYSCFG->CFGR3 & (0x1 << 26))){
      tries--;
      if(tries == 0){
        return -1;
      }
    }
  }
#endif

#ifdef STM32F0
  if(!(RCC->CR & RCC_CR2_HSI48RDY)){
    uint32_t tries = 0x2000000;
    RCC->CR2 |= RCC_CR2_HSI48ON;
    while(!(RCC->CR2 & RCC_CR2_HSI48RDY)){
      tries--;
      if(tries == 0){
        return -1;
      }
    }
  }
#else
  if(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)){
    uint32_t tries = 0x2000000;
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while(!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)){
      tries--;
      if(tries == 0){
        return -1;
      }
    }
  }

  RCC->CCIPR |= RCC_CCIPR_HSI48SEL;
#endif 

  CRS->CR |= CRS_CR_AUTOTRIMEN;
#endif /* ifdef STM32F1XX */
  return 0;
}

#ifdef STM32F1XX
void usb_handler();

void usb_lp_handler(){
  usb_handler();
}

void usb_hp_handler(){
  usb_handler();
}
#endif

void usb_nvic_init(void){
#ifdef STM32F1XX
  NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
#else
  NVIC_EnableIRQ(USB_IRQn);
#endif
}

void usb_nvic_deinit(void){
#ifdef STM32F1XX
  NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
  NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
#else
  NVIC_DisableIRQ(USB_IRQn);
#endif
}

int usb_bsp_init(void){
  if(usb_clk_init()){
    return -1;
  }
  usb_nvic_init();
  return 0;
}
