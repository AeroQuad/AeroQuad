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
  #define FRONTMOTORPIN  2
  #define REARMOTORPIN   3
  #define RIGHTMOTORPIN  5
  #define LEFTMOTORPIN   6
  #define LASTMOTORPIN   7
#else
  #define FRONTMOTORPIN  3
  #define REARMOTORPIN   9
  #define RIGHTMOTORPIN 10
  #define LEFTMOTORPIN  11
  #define LASTMOTORPIN  12
#endif

class Motors_PWM : public Motors {
public:

  Motors_PWM();
	
  void initialize();
  void write();
  void commandAllMotors(int command);
  
};

#endif