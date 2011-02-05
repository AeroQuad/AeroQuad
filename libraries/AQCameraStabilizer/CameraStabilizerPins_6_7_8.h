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

#ifndef _CAMERA_STABILIZER_PINS_6_7_8_H_
#define _CAMERA_STABILIZER_PINS_6_7_8_H_

#include <Camera.h>

class CameraStabilizerPins_6_7_8 : public CameraStabilizer {
public:
  CameraStabilizerPins_6_7_8() : CameraStabilizer() {}
  void _initialize(void) {
    // Init PWM Timer 4    Probable conflict with AeroQuad Motor or Arducopter PPM
    DDRH = DDRH | B00111000;                                  //Set to Output Mega Port-Pin PH3-8, PE4-7, PE5-6
    TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)); 
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = 39999; //50hz freq (standard servos)
  }
  
  void move(void) {
    if (mode > 0) {
      OCR4A = servoPitch * 2;
      OCR4B = servoRoll * 2;
      OCR4C = servoYaw * 2;
    }
  }
};

#endif