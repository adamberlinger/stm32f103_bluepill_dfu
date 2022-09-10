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
#include "systick.h"
#include "arm_target.h"

#define MS_TO_TIMER					(get_core_clock() / 1000)

static volatile uint32_t ms_ticks;

void init_ms_ticks(void){
	SysTick->LOAD = MS_TO_TIMER;
	// ENABLE + SOURCE + INTERRUPT
	ms_ticks = 0;
	SysTick->CTRL = 0x7;

	NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 2);
}

void systick_handler(void){
	ms_ticks++;
}

uint32_t get_ms_ticks(){
	return ms_ticks;
}

void wait_ms(uint32_t ms){
    uint32_t start = get_ms_ticks();
    while((get_ms_ticks() - start) < ms)
        continue;
}

uint32_t sync_ms(uint32_t start,uint32_t ms){
    uint32_t end;
    while(((end = get_ms_ticks()) - start) < ms)
        continue;
    return end;
}
