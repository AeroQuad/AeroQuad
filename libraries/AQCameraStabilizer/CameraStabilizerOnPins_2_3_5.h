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

#ifndef _AQ_CAMERA_STABILIZER_ON_PIN_2_3_5_H_
#define _AQ_CAMERA_STABILIZER_ON_PIN_2_3_5_H_

#include "CameraStabilizer.h"

class CameraStabilizerOnPins_2_3_5 : public CameraStabilizer 
{
public:
  Camera_Pins_2_3_5() : CameraStabilizer() {}
  
  void _initialize() 
  {
    // Init PWM Timer 3    Probable conflict with AeroQuad Motor
    DDRE = DDRE | B00111000;                                  //Set to Output Mega Port-Pin PE4-2, PE5-3, PE3-5
    TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
    ICR3 = 39999; //50hz freq (standard servos)
  }
  
  void move() 
  {
    if (_mode > 0) 
    {
      OCR3A = _servoPitch * 2;
      OCR3B = _servoRoll * 2;
      OCR3C = _servoYaw * 2;
    }
  }
};

#endif