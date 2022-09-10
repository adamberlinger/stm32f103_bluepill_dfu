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

MEMORY
{
  rom2 (rx) : ORIGIN = 0x08000004, LENGTH = 4
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  rom (rx)  : ORIGIN = 0x0801F000, LENGTH = 4K
}

STACK_SIZE = 0x200;

SECTIONS
{
    .boot_vec :
    {
      *(.boot_vec)
    } >rom2


    _DATAI_BEGIN = LOADADDR(.text_data_sram);

    .text_data_sram :
    {
        KEEP(*(.nvic_vector_core))   /* Vector table */
        KEEP(*(.nvic_vector_device))   /* Vector table */
        . = ALIGN(4);
        *(.text)      /* Program code */
        *(.text.*)
        *(.rodata)    /* Read only data */
        *(.rodata.*)
        *(.glue_7)         /* glue arm to thumb code */
        *(.glue_7t)        /* glue thumb to arm code */
    	*(.eh_frame)
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        *(.ARM.exidx*)
        *(.data)      /* Data memory */
        *(.data.*)
        . = ALIGN(4);
        _DATA_END = .;
    } >ram AT >rom

    .startup :
    {
        *(.startup)
    } >rom

    .bss :
    {
        _BSS_BEGIN = .;
        *(.bss)       /* Zero-filled run time allocate data memory */
        *(COMMON)
        _BSS_END = .;
    } >ram

    .stack :
    {
        . += STACK_SIZE;
        _STACKTOP = .;
    } >ram

}
