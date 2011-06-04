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


#ifndef _AEROQUAD_MOTORS_PWM_H_
#define _AEROQUAD_MOTORS_PWM_H_

#include <WProgram.h>

#include "Motors.h"

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MOTORPIN0    2
  #define MOTORPIN1    3
  #define MOTORPIN2    5
  #define MOTORPIN3    6
  #define MOTORPIN4    7
  #define MOTORPIN5    8
#else
  #define MOTORPIN0    3
  #define MOTORPIN1    9
  #define MOTORPIN2   10
  #define MOTORPIN3   11
#endif

class Motors_PWM : public Motors {
public:

  Motors_PWM() {
  }

  void initialize(NB_Motors numbers) {
    commandAllMotors(1000);
  }

  void write() {
    analogWrite(MOTORPIN0, motorCommand[MOTOR1] / 8);
    analogWrite(MOTORPIN1, motorCommand[MOTOR2]  / 8);
    analogWrite(MOTORPIN2, motorCommand[MOTOR3] / 8);
    analogWrite(MOTORPIN3, motorCommand[MOTOR4]  / 8); 
  }

  void commandAllMotors(int command) {
    analogWrite(MOTORPIN0, command / 8);
    analogWrite(MOTORPIN1, command / 8);
    analogWrite(MOTORPIN2, command / 8);
    analogWrite(MOTORPIN3, command / 8);
  }  
};

#endif