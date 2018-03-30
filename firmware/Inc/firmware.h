/*
 * firmware.h
 *
 *  Created on: 30.03.2018
 *      Author: Martin Fees

BSD 3-Clause License

Copyright (c) 2018, FRANC0R - Franconian Open Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef FIRMWARE_H_
#define FIRMWARE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* Public definitions --------------------------------------------------------*/
#define true                    1
#define false                   0

#define FW_RSLT_ERROR           0
#define FW_RSLT_OK              1
#define FW_RSLT_OVFLW           2

#define IMU_TASK_IDLE_TIME_MS   10
#define LED_TASK_IDLE_TIME_MS   500
#define CDC_TASK_IDLE_TIME_MS   1

/* Public function prototypes ------------------------------------------------*/
void imuMain(void);
void ledMain(void);
void cdcMain(void);

void VCP_write(const uint8_t *data, const uint16_t length);
void VCP_print(const char *str);

#endif /* FIRMWARE_H_ */
