/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

#include "Motors_I2C.h"

#include <Device_I2C.h>

#define MOTORBASE 0x28            // I2C controller base address
#define MOTOR_ADDR_0  MOTORBASE + 1  // define I2C controller addresses per your configuration
#define MOTOR_ADDR_1  MOTORBASE + 3  // these addresses are for Phifun controllers
#define MOTOR_ADDR_2  MOTORBASE + 2  // as installed on jihlein's homebrew AeroQuad 3.0
#define MOTOR_ADDR_3  MOTORBASE + 4  // inspired frame
#define MOTOR_ADDR_4  MOTORBASE + 5
#define MOTOR_ADDR_5  MOTORBASE + 6


Motors_I2C::Motors_I2C() {
  motorAddress[0] = MOTOR_ADDR_0;
  motorAddress[1] = MOTOR_ADDR_1;
  motorAddress[2] = MOTOR_ADDR_2;
  motorAddress[3] = MOTOR_ADDR_3;
  motorAddress[4] = MOTOR_ADDR_4;
  motorAddress[5] = MOTOR_ADDR_5;
}

void Motors_I2C::initialize(NB_Motors numbers) {
  lastMotor = numbers;
  for (byte motor = 0; motor < lastMotor; motor++)
    sendByteI2C(motorAddress[motor], 0);
}

void Motors_I2C::write() {
  for (byte motor = 0; motor < lastMotor; motor++)
    sendByteI2C(motorAddress[motor], constrain((motorCommand[motor] - 1000) / 4, 0, 255));
}

void Motors_I2C::commandAllMotors(int command) {
  for (byte motor = 0; motor < lastMotor; motor++)
    sendByteI2C(motorAddress[motor], constrain((motorCommand[motor] - 1000) / 4, 0, 255));
}
