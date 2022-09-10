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

int main(void);
void init_c(void);

void reset_handler(void);

void nmi_handler(void) __attribute__((weak));
void hardfault_handler(void) __attribute__((weak));
void memmanage_handler(void) __attribute__((weak));
void busfault_handler(void) __attribute__((weak));
void usagefault_hander(void) __attribute__((weak));
void svc_handler(void) __attribute__((weak));
void debugmonitor_handler(void) __attribute__((weak));
void pendsv_handler(void) __attribute__((weak));
void systick_handler(void) __attribute__((weak));

extern uint32_t* _STACKTOP;

void* boot_vec[1]
__attribute__ ((section(".boot_vec"))) = {
        (void*) reset_handler,
};

void* the_nvic_vector[16]
__attribute__ ((section(".nvic_vector_core")))= {
        (void*)0x20004FF8,
        (void*) reset_handler,
        (void*) nmi_handler,
        (void*) hardfault_handler,
        (void*) memmanage_handler,
        (void*) busfault_handler,
        (void*) usagefault_hander,
        (void*)0,
        (void*)0,
        (void*)0,
        (void*)0,
        (void*) svc_handler,
        (void*) debugmonitor_handler,
        (void*)0,
        (void*) pendsv_handler,
        (void*) systick_handler
};

__attribute__((section(".startup")))
void reset_handler_c(void){
    init_c();
    main();
}

void nmi_handler(void){}
void hardfault_handler(void){}
void memmanage_handler(void){}
void busfault_handler(void){}
void usagefault_hander(void){}
void svc_handler(void){}
void debugmonitor_handler(void){}
void pendsv_handler(void){}
void systick_handler(void){}
