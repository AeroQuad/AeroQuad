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

/******************************************************/
/********************* Fake PWM Motors ****************/
/******************************************************/

#ifndef _AQ_CHR6DM_FAKE_MOTORS_H_
#define _AQ_CHR6DM_FAKE_MOTORS_H_

#include "Motors.h"

class CHR6DMFakeMotors : public Motors 
{
private:

  int minCommand;
  byte pin;

 public:
  CHR6DMFakeMotors() : Motors()
  {
    // Scale motor commands to analogWrite
    // Only supports commands from 0-255 => 0 - 100% duty cycle
    // Usable pulsewith from approximately 1000-2000 us = 126 - 250
    // m = (250-126)/(2000-1000) = 0.124
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2
    _mMotorCommand = 0.124;
    _bMotorCommand = 2.0;
  }

  void initialize() 
  {
    pinMode(FRONTMOTORPIN, OUTPUT);
    fake_analogWrite(FRONTMOTORPIN, 124);
    pinMode(REARMOTORPIN, OUTPUT);
    fake_analogWrite(REARMOTORPIN, 124);
    pinMode(RIGHTMOTORPIN, OUTPUT);
    fake_analogWrite(RIGHTMOTORPIN, 124);
    pinMode(LEFTMOTORPIN, OUTPUT);
  }

  void write() 
  {
    fake_analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * _mMotorCommand) + _bMotorCommand);
    fake_analogWrite(REARMOTORPIN, (motorCommand[REAR] * _mMotorCommand) + _bMotorCommand);
    fake_analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * _mMotorCommand) + _bMotorCommand);
    fake_analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * _mMotorCommand) + _bMotorCommand);
  }

  void commandAllMotors(int _motorCommand) 
  {   // Sends commands to all motors
    fake_analogWrite(FRONTMOTORPIN, (_motorCommand * _mMotorCommand) + _bMotorCommand);
    fake_analogWrite(REARMOTORPIN, (_motorCommand * _mMotorCommand) + _bMotorCommand);
    fake_analogWrite(RIGHTMOTORPIN, (_motorCommand * _mMotorCommand) + _bMotorCommand);
    fake_analogWrite(LEFTMOTORPIN, (_motorCommand * _mMotorCommand) + _bMotorCommand);
  }

  void fake_analogWrite(int pin, int value)
  {
    Serial2.print("analogWrite:");
    Serial2.print(pin);
    Serial2.print(",");
    Serial2.println(value);
  }
};

#endif