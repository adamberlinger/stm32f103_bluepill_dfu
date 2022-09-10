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
#ifndef _USB_DESC_H_
#define _USB_DESC_H_

#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

#define ALLOW_OPTION_BYTES    (0)

#define DEVICE_DESCRIPTOR_SIZE          (18)
#if ALLOW_OPTION_BYTES
#define CONFIG_DESCRIPTOR_SIZE          (0x24)
#else
#define CONFIG_DESCRIPTOR_SIZE          (0x1B)
#endif

#define SIZE_TO_U16(size)               ((size+1)/2)

extern const uint16_t device_descriptor[SIZE_TO_U16(DEVICE_DESCRIPTOR_SIZE)];
extern const uint16_t config_descriptor[SIZE_TO_U16(CONFIG_DESCRIPTOR_SIZE)];

extern const uint16_t string_descriptor0[4];

#if ALLOW_OPTION_BYTES
#define STRING_DESCRIPTOR_COUNT (5)
#else
#define STRING_DESCRIPTOR_COUNT (4)
#endif

extern const char* string_descriptors[STRING_DESCRIPTOR_COUNT];


#ifdef __cplusplus
    }
#endif

#endif /* _USB_DESC_H_ */
