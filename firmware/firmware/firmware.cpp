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

inline float rangeAngles(const float& angle, const float& max_angle_abs) {
  if(angle > max_angle_abs) {
    return max_angle_abs;
  }
  else if(angle < -max_angle_abs){
    return -max_angle_abs;
  }

  return angle;
}


LevelingPlatform::LevelingPlatform(const PinName& board_led, const PinName& imu_sda,
                                    const PinName& imu_scl):
_servo1(PA_8),
_servo2(PA_9)
{
  // config system clock
  confSysClock();

  // Create instance of board led pin
  _board_led = new DigitalOut(board_led);

  // Create instance of the IMU
  _imu       = new BNO055(imu_sda, imu_scl);

  // Create instance of the USB virtual com port (CDC)
  _vcp       = new USBSerial(0x1f00, 0x2012, 0x0001,  false);

  // Reset variables
  _tick_cnt = 0;
  _vcp_available = false;

  // Setup frequency
  _servo1.period_ms(20);
  _servo2.period_ms(20);

  // Init VCP
  if(_initVCP() != FW_OK) {
      //TODO error handler
  }

  // Init IMU
  if(_initIMU() != FW_OK) {
      //TODO error handler
  }
}

FwResults LevelingPlatform::run(void) {
  uint32_t tmp_tick_imu = 0;
  uint32_t tmp_tick_vcp = 0;
  uint32_t tmp_tick_servo = 0;
  int32_t tmp_pw1 = 0;
  int32_t tmp_pw2 = 0;
  float angle1, angle2;
  int32_t pw1, pw2;

  while(true) {
    if((_tick_cnt - tmp_tick_imu) > 10) {
      _imu->get_angles();
      tmp_tick_imu = _tick_cnt;
    }

    if((_tick_cnt - tmp_tick_vcp) > 100) {
      if((_vcp->isConnected() == true) &&
         (_vcp->available() == true) &&
         (_vcp_available == false)) {
            _vcp_available = true;
         }

      tmp_tick_vcp = _tick_cnt;
    }

    if((_tick_cnt - tmp_tick_servo) > 20) {
      angle1 = (rangeAngles(_imu->euler.roll, 90.0f) / 90.0f) * 1000.0f + 1500.0f;
      angle2 = (rangeAngles(_imu->euler.pitch, 90.0f) / 90.0f) * 1000.0f + 1500.0f;

      pw1 = angle1;
      pw2 = angle2;

      if(abs(pw1 - tmp_pw1) > 5) {
        tmp_pw1 = pw1;
        _servo1.pulsewidth_us(pw1);
      }
      else if(abs(pw2 - tmp_pw2) > 5) {
        tmp_pw2 = pw2;
        _servo2.pulsewidth_us(pw2);
      }

      printInfo("Angle1: %i Angle2: %i", pw1, pw2);

      tmp_tick_servo = _tick_cnt;
    }


    _tick_cnt++;
    wait_ms(1);
  }
}

void LevelingPlatform::printInfo(const char* fmt, ...) {
  if(_vcp_available == false) return;

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

    return FW_OK;
}
