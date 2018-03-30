/*
 * firmware.c
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

#include "main.h"
#include "firmware.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Private function prototypes -----------------------------------------------*/

/* FreeRTOS task functions ---------------------------------------------------*/

/**
 *  @brief Main function for handling IMU via I2C
 */
void imuMain(void)
{
}

/**
 *  @brief Main function for blinking board LED
 */
void ledMain(void)
{
  HAL_GPIO_TogglePin(BOARD_LED_GPIO_Port, BOARD_LED_Pin);
}

/**
 * @brief Main function for handling CDC or virtual com port via USB
 */
void cdcMain(void)
{
  /* Send received data back to transmitter for testing */
  if(CDC_RXData.NewData) {
    VCP_write(CDC_RXData.Data, CDC_RXData.Length);
    CDC_Data_Readed();
  }
}

/* Virtual Com Port functions ------------------------------------------------*/

/**
 * @brief Transmits data with a specific length via CDC
 *
 * @param data Pointer to the data buffer
 * @param length Length of the data
 */
void VCP_write(const uint8_t *data, const uint16_t length)
{
    CDC_Transmit_FS((uint8_t*)data, length);
}

/**
 * @brief Transmits a string via CDC
 *
 * @param str Pointer to a string
 */
void VCP_print(const char *str)
{
  char buffer[256];

  /* Copy string to buffer with escape sequence */
  sprintf(buffer, "%s\r\n", str);

  /* Transmit data via CDC */
  CDC_Transmit_FS((uint8_t*)buffer, (uint16_t)strlen(buffer));
}

