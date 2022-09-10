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

#include "stm32_common.h"
#include "usb_core.h"
#include "systick.h"
#include "usb_dfu.h"

uint32_t core_clock = 8000000;

uint32_t get_core_clock(){
  return core_clock;
}

int init_cpu(void){
    int tries = 2000;
    int status = 0;
    uint32_t tmpreg;

    int use_hse = 1;

    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    if(use_hse){
        #ifdef TARGET_HSE_BYP
            RCC->CR |= RCC_CR_HSEBYP;
        #else
            RCC->CR &= ~(uint32_t)RCC_CR_HSEBYP;
        #endif
        RCC->CR |= RCC_CR_HSEON;

        while(tries && (RCC->CR & RCC_CR_HSERDY) == 0){
            tries--;
        }

        if((RCC->CR & RCC_CR_HSERDY) == 0){
            /* Fall back to internal oscillator */
            use_hse = 0;
            status = -2;
        }
    }

    /* TODO: compute PLL parameters */
    /* TODO: make work for HSE */
    /* PLLMUL = 9*/
    /* Set AHB & APB prescalers */
    if(use_hse){
        tmpreg = RCC_CFGR_PLLMULL9 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PLLSRC;
    }
    else {
        tmpreg = RCC_CFGR_PLLMULL16 | RCC_CFGR_PPRE1_DIV2;
    }
    RCC->CFGR = tmpreg;

    RCC->CR |= RCC_CR_PLLON;

    tries = 2000;
    while(tries && (RCC->CR & RCC_CR_PLLRDY) == 0){
        tries--;
    }

    if((RCC->CR & RCC_CR_PLLRDY) == 0){
        return -1;
    }
    else {
        /* Set flash latency for higher core clock */
        tmpreg = FLASH->ACR;
        tmpreg &= ~FLASH_ACR_LATENCY;
        /* Set 2 wait-states */
        tmpreg |= (2);
        FLASH->ACR |= tmpreg;

        /* Switch to PLL clocks */
        tmpreg = RCC->CFGR;
        tmpreg &= ~RCC_CFGR_SW;
        tmpreg |= RCC_CFGR_SW_PLL;
        RCC->CFGR = tmpreg;

        if(use_hse){
            core_clock = 72000000;
        }
        else {
            core_clock = 64000000;
        }
    }

    return status;
}

void main(){

  init_cpu();
  init_ms_ticks();

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN;
  /* PA12 to open drain */
  GPIOA->CRH = (GPIOA->CRH & 0xFFF0FFFF) | 0x00070000;
  GPIOA->BRR = (1 << 12);
  wait_ms(10);
  /* PA12 to input floating */
  GPIOA->CRH = (GPIOA->CRH & 0xFFF0FFFF) | 0x00040000;

  GPIOC->CRH = (GPIOA->CRH & 0xFF0FFFFF) | 0x00700000;
  GPIOC->BSRR = (1 << 13);

  /* Unlock the flash */
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;

  usb_core_init();

  while(1){
    usb_dfu_main_thread();
  }
}
