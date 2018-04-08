/*
 * firmware.h
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

#ifndef FIRMWARE_H_
#define FIRMWARE_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f103c8t6.h"
#include "mbed.h"
#include "BNO055.h"
#include "USBSerial.h"
#include "Serial.h"

/* Definitions ---------------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/* Enumerations --------------------------------------------------------------*/

enum FwResults {
    FW_ERROR = 0,
    FW_OK    = 1
};

/* Classes and Structs -------------------------------------------------------*/

class LevellingPlatform
{
 public:
  /**
  * @brief Levelling Platform Constructor
  */
  LevellingPlatform(const PinName& board_led, const PinName& imu_sda,
                     const PinName& imu_scl);

  /**
   * @brief Prints a infor via the virtual com port
   */
  void PrintInfo(const char* fmt);
  
  const BNO055& GetIMU()    {return (*_imu);}
    
 private:
 
  /**
   * @brief Initializes the IMU
   */
  FwResults _InitIMU();
  
  DigitalOut*       _board_led; //!< Board LED
  USBSerial*        _vcp;       //!< Virtual com port
  BNO055*           _imu;       //!< IMU - Inertial Mesaurement Unit
  
};

#endif /* FIRMWARE_H_ */