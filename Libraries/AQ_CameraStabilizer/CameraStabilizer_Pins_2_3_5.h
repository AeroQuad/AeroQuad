/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AEROQUAD_CAMERA_STABILIZER_2_3_5_H_
#define _AEROQUAD_CAMERA_STABILIZER_2_3_5_H_


#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // used only on mega for now

// Written by CupOfTea:
// http://aeroquad.com/showthread.php?1484-Camera-Stablisation
// http://aeroquad.com/showthread.php?1491-Camera-Stablisation-continued

#include "CameraStabilizer.h"

void initializeCameraControl() {
  // Init PWM Timer 3    Probable conflict with AeroQuad Motor
  DDRE = DDRE | B00111000;                                  //Set to Output Mega Port-Pin PE4-2, PE5-3, PE3-5
  TCCR3A =((1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1));
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
  ICR3 = 39999; //50hz freq (standard servos)
}
  
void cameraControlMove() {
  if (mode > 0) {
    OCR3A = servoPitch * 2;
    OCR3B = servoRoll * 2;
    OCR3C = servoYaw * 2;
  }
}



#endif  // #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#endif  // #define _AEROQUAD_CAMERA_STABILIZER_H_
