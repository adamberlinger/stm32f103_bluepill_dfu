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
#include <stdint.h>
#include "arm_target.h"

extern uint32_t _DATA_BEGIN;
extern uint32_t _DATA_END;
extern uint32_t _DATAI_BEGIN;
extern uint32_t _DATAI_END;
extern uint32_t _BSS_BEGIN;
extern uint32_t _BSS_END;

__attribute__ ((section(".startup")))
void init_c(void){

#ifdef __GNUC__


    uint32_t* clr_begin  = (uint32_t*)0x20000000;
    uint32_t* clr_end = (uint32_t*)0x20005000;
    while(clr_begin < clr_end){
      *clr_begin = 0xA5A5A5A5;
      clr_begin++;
    }

    uint32_t* bss_begin = (uint32_t*)&_BSS_BEGIN;
    uint32_t* bss_end   = (uint32_t*)&_BSS_END;
    while(bss_begin < bss_end)
    {
        *bss_begin = 0;
        bss_begin++;
    }

    uint32_t* data_begin  = (uint32_t*)0x20000000;
    uint32_t* data_end    = (uint32_t*)&_DATA_END;
    uint32_t* datai_begin = (uint32_t*)&_DATAI_BEGIN;

    while(data_begin < data_end)
    {
        *data_begin = *datai_begin;
        data_begin++;
        datai_begin++;
    }
  	__DSB();
    __ISB();
  	SCB->VTOR = 0x20000000;
    __DSB();
  	__ISB();
#endif

#ifdef __FPU_USED
    /* enable FPU unit */
    uint32_t *cpacr = (uint32_t*)0xE000ED88;
    *cpacr |= (0xF << 20);
    __DSB();
    __ISB();
#endif
}
