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
#ifndef _USB_DFU_H_
#define _USB_DFU_H_

#include <stdint.h>

#ifdef __cplusplus
    extern "C" {
#endif

void usb_dfu_request(const usb_request_t *request, const uint8_t *request_data);
void usb_dfu_enable(void);
void usb_dfu_data_received(uint16_t* data, uint16_t length);

uint8_t usb_dfu_rx_ready(void);

#define DFU_DETACH    (0x00)
#define DFU_DNLOAD    (0x01)
#define DFU_UPLOAD    (0x02)
#define DFU_GETSTATUS (0x03)
#define DFU_CLRSTATUS (0x04)
#define DFU_GETSTATE  (0x05)
#define DFU_ABORT     (0x06)

#define dfuIDLE       (2)
#define dfuDNBUSY     (4)
#define dfuDNLOAD_IDLE (5)
#define dfuUPLOAD_IDLE (9)
#define dfuERROR      (10)

#define errTARGET     (1)

typedef struct {
  uint8_t status;
  uint8_t poll_timeout[3];
  uint8_t state;
  uint8_t string;
}dfu_status_t;

typedef enum {
  DFU_MODE_NONE = dfuIDLE,
  DFU_MODE_DOWNLOAD = dfuDNLOAD_IDLE,
  DFU_MODE_UPLOAD = dfuUPLOAD_IDLE,
}dfu_mode_t;

typedef enum {
  DFU_CMD_IDLE,
  DFU_CMD_PARSE,
  DFU_CMD_ERASE,
  DFU_CMD_WRITE_PARSE,
  DFU_CMD_WRITE
}dfu_cmd_status_t;

#define DFU_APP_RESET_HANDLER   (0x0800001C)

void usb_dfu_main_thread(void);

#ifdef __cplusplus
    }
#endif

#endif /* _USB_DESC_H_ */
