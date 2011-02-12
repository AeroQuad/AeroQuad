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

#ifndef _AQ_CAMERA_STABILIZER_ON_PIN_44_45_46_H_
#define _AQ_CAMERA_STABILIZER_ON_PIN_44_45_46_H_

#include "CameraStabilizer.h"

class CameraStabilizerOnPins_44_45_46 : public CameraStabilizer 
{
public:
  CameraStabilizerOnPins_44_45_46() : CameraStabilizer() {}
  
  void _initialize() 
  {
    // Init PWM Timer 5   Probable conflict with Arducopter Motor
    DDRL = DDRL | B00111000;                                  //Set to Output Mega Port-Pin PL3-46, PE4-45, PE5-44
    TCCR5A =((1<<WGM51)|(1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)); 
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    ICR5 = 39999; //50hz freq (standard servos)
  }
  
  void move() 
  {
    if (_mode > 0) 
    {
      OCR5A = _servoPitch * 2;
      OCR5B = _servoRoll * 2;
      OCR5C = _servoYaw * 2;      
    }
  }
};

#endif