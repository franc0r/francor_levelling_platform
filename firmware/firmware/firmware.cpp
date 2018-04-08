/*
 * firmware.cpp
 *
 *  Created on: 08.04.2018
 *      Author: feesmrt
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, FRANC0R - Franconian Open Robotics
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
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

/* Includes ------------------------------------------------------------------*/
#include "firmware.h"

/* Private Functions----------------------------------------------------------*/

LevelingPlatform::LevelingPlatform(const PinName& board_led, const PinName& imu_sda,
                                    const PinName& imu_scl)
{
  // config system clock
  confSysClock();

  // Create instance of board led pin
  _board_led = new DigitalOut(board_led);

  // Create instance of the IMU
  _imu       = new BNO055(imu_sda, imu_scl);

  // Create instance of the USB virtual com port (CDC)
  _vcp       = new USBSerial(0x1f00, 0x2012, 0x0001,  false);

  // Init VCP
  if(_initVCP() != FW_OK) {
      //TODO error handler
  }

  // Init IMU
  if(_initIMU() != FW_OK) {
      //TODO error handler
  }
}

void LevelingPlatform::printInfo(const char* fmt, ...) {
  // variables
  va_list args;
  char buf[FW_MAX_BUFFER_SIZE];

  // modify string and ad sts + \r\n at end
  sprintf(buf, "xInfo: %s\r\n", fmt);
  buf[0] = 0x02; // STS - start of text

  // push via CDC
  va_start(args, fmt);
  _vcp->vprintf(buf, args);
  va_end(args);
}

FwResults LevelingPlatform::_initVCP() {
  // Wait for terminal connection
  while(_vcp->isConnected() == false) {
      (*_board_led) = 0;
      wait_ms(500);
      (*_board_led) = 1;
      wait_ms(500);
  }

  // Wait for first byte
  while(_vcp->available() == 0) {
      (*_board_led) = 0;
      wait_ms(500);
      (*_board_led) = 1;
      wait_ms(500);
  }

  return FW_OK;
}

FwResults LevelingPlatform::_initIMU() {
  // Reset IMU
  _imu->reset();

  // Check if IMU is responding
  if( !_imu->check() ) {
      //TO DO error handler
  }

  // activate external crystal
  _imu->SetExternalCrystal(true);

  // setup operation mode
  _imu->setmode(OPERATION_MODE_NDOF);

  printInfo("Adafruit IMU BNO055\r\n");
  printInfo("Serial:\r\n");

  // print serial number
  for(uint8_t idx = 0; idx < 4; idx++) {
    printInfo("%0x.%0x.%0x.%0x\r\n",_imu->ID.serial[idx*4],
              _imu->ID.serial[idx*4+1],
              _imu->ID.serial[idx*4+2],
              _imu->ID.serial[idx*4+3]);
  }

    return FW_OK;
}
