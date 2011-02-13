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

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include "CameraStabilizerOnPins_6_7_8.h"

CameraStabilizerOnPins_6_7_8::CameraStabilizerOnPins_6_7_8()
{
}
  
void CameraStabilizerOnPins_6_7_8::_initialize() 
{
  // Init PWM Timer 4    Probable conflict with AeroQuad Motor or Arducopter PPM
  DDRH = DDRH | B00111000;                                  //Set to Output Mega Port-Pin PH3-8, PE4-7, PE5-6
  TCCR4A =((1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)); 
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = 39999; //50hz freq (standard servos)
}
  
void CameraStabilizerOnPins_6_7_8::move() 
{
  if (_mode > 0) 
  {
    OCR4A = _servoPitch * 2;
    OCR4B = _servoRoll * 2;
    OCR4C = _servoYaw * 2;
  }
}

#endif
