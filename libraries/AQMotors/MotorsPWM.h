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

#ifndef _MOTORS_PWM_H_
#define _MOTORS_PWM_H_

#include <Motors.h>

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define FRONTMOTORPIN  3
  #define REARMOTORPIN   9
  #define RIGHTMOTORPIN 10
  #define LEFTMOTORPIN  11
  #define LASTMOTORPIN  12
#else
  #define FRONTMOTORPIN  2
  #define REARMOTORPIN   3
  #define RIGHTMOTORPIN  5
  #define LEFTMOTORPIN   6
  #define LASTMOTORPIN   7
#endif

/******************************************************/
/********************* PWM Motors *********************/
/******************************************************/
class MotorsPWM : public Motors {
private:
  
  int minCommand;
  byte pin;

 public:
  MotorsPWM() : Motors(){
   // Analog write supports commands from 0-255 => 0 - 100% duty cycle
   // Using 125-250 for motor setting 1000-2000
  }

  void initialize(void) {
    commandAllMotors(1000);
  }

  void write(void) {
    analogWrite(FRONTMOTORPIN, motorCommand[FRONT] / 8);
    analogWrite(REARMOTORPIN,  motorCommand[REAR]  / 8);
    analogWrite(RIGHTMOTORPIN, motorCommand[RIGHT] / 8);
    analogWrite(LEFTMOTORPIN,  motorCommand[LEFT]  / 8);

  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, _motorCommand / 8);
    analogWrite(REARMOTORPIN,  _motorCommand / 8);
    analogWrite(RIGHTMOTORPIN, _motorCommand / 8);
    analogWrite(LEFTMOTORPIN,  _motorCommand / 8);
  }
};

#endif