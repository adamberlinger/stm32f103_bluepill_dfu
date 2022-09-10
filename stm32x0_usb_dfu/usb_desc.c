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
#include "usb_desc.h"

#define U8_TO_U16(a,b)          (((uint16_t)a) | ((uint16_t)b << 8))

/*
 * This is nasty trick to keep descriptors 16-bit aligned on all compilers.
 * But there is usually no need to modify these
 */

const uint16_t device_descriptor[9] = {
  U8_TO_U16(18,                   /* Size */
  0x01),                 /* Device descriptor */
  U8_TO_U16(0x00, 0x01),           /* USB 2.0 */
  U8_TO_U16(0x00,                 /* Device class: CDC */
  0x00),                 /* Device subclass: CDC */
  U8_TO_U16(0x00,                 /* Device protocol */
  64),                   /* Max. packet size */
  U8_TO_U16(0x83, 0x04),           /* VID: 0x0483 */
  U8_TO_U16(0x11, 0xDF),           /* PID: 0x5740 */
  U8_TO_U16(0x00, 0x22),           /* Device version 1.0 */
  U8_TO_U16(0x01,                 /* Manufacturer string */
  0x02),                 /* Product string */
  U8_TO_U16(0x03,                 /* Serial number */
  0x01),                 /* Number of configurations */
};

const uint16_t config_descriptor[SIZE_TO_U16(CONFIG_DESCRIPTOR_SIZE)] = {
  U8_TO_U16(9,                    /* Size */
  0x02),                 /* Configuration descriptor */
  CONFIG_DESCRIPTOR_SIZE,         /* Total length */
  U8_TO_U16(0x01,                 /* Number of interfaces */
  0x01),                 /* Configuration value */
  U8_TO_U16(0x00,                 /* Configuration index */
  0x80),
  U8_TO_U16(50,                   /* 100mA consumption */

  /*Interface Descriptor */
  0x09),   /* bLength: Interface Descriptor size */
  U8_TO_U16(0x04,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00),   /* bInterfaceNumber: Number of Interface */
  U8_TO_U16(0x00,   /* bAlternateSetting: Alternate setting */
  0x00),   /* bNumEndpoints: No endpoints used */
  U8_TO_U16(0xFE,   /* bInterfaceClass: Communication Interface Class */
  0x01),   /* bInterfaceSubClass: Abstract Control Model */
  U8_TO_U16(0x02,   /* bInterfaceProtocol: Common AT commands */
  0x04),   /* iInterface: */

#if ALLOW_OPTION_BYTES
  /*Interface Descriptor */
  U8_TO_U16(0x09,   /* bLength: Endpoint Descriptor size */
  0x04),   /* bDescriptorType: CS_INTERFACE */
  U8_TO_U16(0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x01),   /* bcdCDC: spec release number */
  U8_TO_U16(0x00,
  0xFE),   /* bFunctionLength */
  U8_TO_U16(0x01,   /* bDescriptorType: CS_INTERFACE */
  0x02),   /* bDescriptorSubtype: Call Management Func Desc */
  U8_TO_U16(0x05,   /* bmCapabilities: D0+D1 */


  /* DFU descriptor */
  0x09),
  U8_TO_U16(0x21,
  0x0B),
  U8_TO_U16(0xFF,
  0x00),
  U8_TO_U16(0x40,
  0x00),
  U8_TO_U16(0x1A,
  0x01),
#else
  /* DFU descriptor */
  U8_TO_U16(0x09,
  0x21),
  U8_TO_U16(0x0B,
  0xFF),
  U8_TO_U16(0x00,
  0x40),
  U8_TO_U16(0x00,
  0x1A),
  U8_TO_U16(0x01,

  0x00)
#endif

};

const uint16_t string_descriptor0[4] = {
  U8_TO_U16(0x04, 0x03), 0x0409
};

const char* string_descriptors[STRING_DESCRIPTOR_COUNT] = {
  "STMicroelectronics",
  "STM32 Bluepill bootloader",
  "FFFFFFFEFFFF",
  "@Internal Flash /0x08000000/124*0001Kg",
#if ALLOW_OPTION_BYTES
  "@Option Bytes /0x1FFFF800/01*016 e"
#endif
};
