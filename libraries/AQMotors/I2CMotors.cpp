/*
  AeroQuad v2.2 - Feburary 2011
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

#include "I2CMotors.h"

#include <I2C.h>

#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3

#define MOTORBASE 0x28            // I2C controller base address
#define FRONTMOTORID MOTORBASE + 1  // define I2C controller addresses per your configuration
#define REARMOTORID  MOTORBASE + 3  // these addresses are for Phifun controllers
#define RIGHTMOTORID MOTORBASE + 2  // as installed on jihlein's homebrew AeroQuad 3.0
#define LEFTMOTORID  MOTORBASE + 4  // inspired frame

I2CMotors::I2CMotors()
{
  // Scale motor commands to 0 to 255
  // for I2C commands
  // m = (255 - 0)/(2000-1000) = 0.255
  // b = y1 - (m * x1) = 0 - (0.255 * 1000) = -255
  _mMotorCommand = 0.255;
  _bMotorCommand = -255.0;
}

void I2CMotors::initialize()
{
  sendByteI2C(FRONTMOTORID, 0);
  sendByteI2C(REARMOTORID,  0);
  sendByteI2C(RIGHTMOTORID, 0);
  sendByteI2C(LEFTMOTORID,  0);
}

void I2CMotors::write()
{
  sendByteI2C(FRONTMOTORID, constrain((_motorCommand[FRONT] * _mMotorCommand) + _bMotorCommand, 0, 255));
  sendByteI2C(REARMOTORID,  constrain((_motorCommand[REAR]  * _mMotorCommand) + _bMotorCommand, 0, 255));
  sendByteI2C(RIGHTMOTORID, constrain((_motorCommand[RIGHT] * _mMotorCommand) + _bMotorCommand, 0, 255));
  sendByteI2C(LEFTMOTORID,  constrain((_motorCommand[LEFT]  * _mMotorCommand) + _bMotorCommand, 0, 255));
}

void I2CMotors::commandAllMotors(int motorCommand)
{
  sendByteI2C(FRONTMOTORID, constrain((motorCommand * _mMotorCommand) + _bMotorCommand, 0, 255));
  sendByteI2C(REARMOTORID,  constrain((motorCommand * _mMotorCommand) + _bMotorCommand, 0, 255));
  sendByteI2C(RIGHTMOTORID, constrain((motorCommand * _mMotorCommand) + _bMotorCommand, 0, 255));
  sendByteI2C(LEFTMOTORID,  constrain((motorCommand * _mMotorCommand) + _bMotorCommand, 0, 255));
}
