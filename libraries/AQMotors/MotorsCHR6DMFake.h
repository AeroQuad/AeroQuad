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

#ifndef _MOTORS_CHR6DM_FAKE_H_
#define _MOTORS_CHR6DM_FAKE_H_

#include <Motors.h>

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define FRONTMOTORPIN 3
  #define REARMOTORPIN 9
  #define RIGHTMOTORPIN 10
  #define LEFTMOTORPIN 11
  #define LASTMOTORPIN 12
#else
  #define FRONTMOTORPIN 2
  #define REARMOTORPIN 3
  #define RIGHTMOTORPIN 5
  #define LEFTMOTORPIN 6
  #define LASTMOTORPIN 7
#endif
  
/******************************************************/
/********************* Fake PWM Motors ****************/
/******************************************************/
class MotorsCHR6DMFake : public Motors {
private:
  
  int minCommand;
  byte pin;

 public:
  MotorsCHR6DMFake() : Motors(){
    // Scale motor commands to analogWrite
    // Only supports commands from 0-255 => 0 - 100% duty cycle
    // Usable pulsewith from approximately 1000-2000 us = 126 - 250
    // m = (250-126)/(2000-1000) = 0.124
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2
    mMotorCommand = 0.124;
    bMotorCommand = 2.0;
  }

  void initialize(void) {
    pinMode(FRONTMOTORPIN, OUTPUT);
    fake_analogWrite(FRONTMOTORPIN, 124);
    pinMode(REARMOTORPIN, OUTPUT);
    fake_analogWrite(REARMOTORPIN, 124);
    pinMode(RIGHTMOTORPIN, OUTPUT);
    fake_analogWrite(RIGHTMOTORPIN, 124);
    pinMode(LEFTMOTORPIN, OUTPUT);
  }

  void write(void) {
    fake_analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);
    fake_analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);
    fake_analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);
    fake_analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);
  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    fake_analogWrite(FRONTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    fake_analogWrite(REARMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    fake_analogWrite(RIGHTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    fake_analogWrite(LEFTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
  }

  void fake_analogWrite(int pin, int value){
    Serial2.print("analogWrite:");
    Serial2.print(pin);
    Serial2.print(",");
    Serial2.println(value);
  }
};

#endif