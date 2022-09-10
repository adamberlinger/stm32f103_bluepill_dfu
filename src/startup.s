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
.syntax unified
.cpu cortex-m3
.thumb

.global	reset_handler
.type reset_handler, %function

.section	.startup

reset_handler:
    /* Enable GPIOB clock */
    LDR r2,=0x40021018
    mov r0,#8
    STR r0,[R2]

    /* Check PB2 */
    LDR r1,=0x40010C08
    LDR r0,[r1]
    ANDS r0, r0, #4
    BNE bootloader

    /* Bootloader not activated */

    /* Disable GPIOB clocks */
    MOV r0,#0
    STR r0,[R2]

    /* Set app stack pointer */
    LDR r1,=0x08000000
    LDR r0, [r1]
    MOV sp, r0

    /* Read user application reset handler */
    LDR r0, [r1, #0x1C]

    /* Continue to bootloader if not valid address */
    AND r2, r0, #0xFF000000
    CMP r2, r1
    BNE bootloader
    BX r0

bootloader:
    LDR r0,=0x20004FF8
    MOV sp, r0

    LDR r0,=reset_handler_c
    BX r0
end_loop:
    b end_loop
