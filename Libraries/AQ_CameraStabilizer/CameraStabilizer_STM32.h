/*
  AeroQuad v3.1.x - August 2012
  www.AeroQuad.com
  Copyright (c) 2012 AeroQuad developers.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AEROQUAD_CAMERA_STABILIZER_STM32_H_
#define _AEROQUAD_CAMERA_STABILIZER_STM32_H_

#if !defined(AeroQuadSTM32)
  #error This code only works on STM32
#endif

#include "CameraStabilizer.h"

#define SERVO_FREQUENCY 50 // 50 Hz for analog servo
//#define SERVO_FREQUENCY 400 // 300 Hz for digital high speed servo

#define SERVO_PERIOD (1000000/SERVO_FREQUENCY)

static byte servopins[3] = {
  Port2Pin('A', 2), // SRV1 -- TIM5CH3
  Port2Pin('A', 1), // SRV2 -- TIM5CH2
  Port2Pin('A', 0), // SRV3 -- TIM5CH1
};


void initializeCameraControl() {

  int servo;

  for ( servo = 0; servo < 3; servo++) {
    int prescaler = rcc_dev_timer_clk_speed(PIN_MAP[servopins[servo]].timer_device->clk_id)/1000000 - 1;
    timer_set_prescaler(PIN_MAP[servopins[servo]].timer_device, prescaler);
    timer_set_reload(PIN_MAP[servopins[servo]].timer_device, SERVO_PERIOD);
    pinMode(servopins[servo], PWM);
  }

  for ( servo = 0; servo < 3; servo++) {
    timer_generate_update(PIN_MAP[servopins[servo]].timer_device);
  }

  cameraControlMove(1500,1500,1500);
}

void cameraControlMove(int servoPitch, int servoRoll, int servoYaw) {

  timer_set_compare(PIN_MAP[servopins[0]].timer_device,
		    PIN_MAP[servopins[0]].timer_channel,
		    servoPitch);
  timer_set_compare(PIN_MAP[servopins[1]].timer_device,
		    PIN_MAP[servopins[1]].timer_channel,
		    servoRoll);
  timer_set_compare(PIN_MAP[servopins[2]].timer_device,
		    PIN_MAP[servopins[2]].timer_channel,
		    servoYaw);
}

#endif  // #define _AEROQUAD_CAMERA_STM32_H_

