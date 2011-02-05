/*
  AeroQuad v2.1 - January 2011
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

#ifndef _MOTORS_I2C_H_
#define _MOTORS_I2C_H_

#include <Motors.h>

/******************************************************/
/********************* I2C Motors *********************/
/******************************************************/
// Tested AeroQuad I2C class authored by jihlein
// http://code.google.com/p/aeroquad/issues/detail?id=67
class MotorsI2C : public Motors {
private:
  #define MOTORBASE 0x28            // I2C controller base address

  #define FRONTMOTORID MOTORBASE + 1  // define I2C controller addresses per your configuration
  #define REARMOTORID  MOTORBASE + 3  // these addresses are for Phifun controllers
  #define RIGHTMOTORID MOTORBASE + 2  // as installed on jihlein's homebrew AeroQuad 3.0
  #define LEFTMOTORID  MOTORBASE + 4  // inspired frame

  public:
  MotorsI2C() : Motors(){
    // Scale motor commands to 0 to 255
    // for I2C commands
    // m = (255 - 0)/(2000-1000) = 0.255
    // b = y1 - (m * x1) = 0 - (0.255 * 1000) = -255
    mMotorCommand = 0.255;
    bMotorCommand = -255.0;
  }

  void initialize(void)
  {
    sendByteI2C(FRONTMOTORID, 0);
    sendByteI2C(REARMOTORID,  0);
    sendByteI2C(RIGHTMOTORID, 0);
    sendByteI2C(LEFTMOTORID,  0);
  }

  void write(void)
  {
    sendByteI2C(FRONTMOTORID, constrain((motorCommand[FRONT] * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(REARMOTORID,  constrain((motorCommand[REAR]  * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(RIGHTMOTORID, constrain((motorCommand[RIGHT] * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(LEFTMOTORID,  constrain((motorCommand[LEFT]  * mMotorCommand) + bMotorCommand, 0, 255));
  }

  void commandAllMotors(int motorCommand)
  {
    sendByteI2C(FRONTMOTORID, constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(REARMOTORID,  constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(RIGHTMOTORID, constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
    sendByteI2C(LEFTMOTORID,  constrain((motorCommand * mMotorCommand) + bMotorCommand, 0, 255));
  }
};

#endif