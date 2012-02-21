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

#ifndef _AEROQUAD_CAMERA_STABILIZER_AEROQUAD_H_
#define _AEROQUAD_CAMERA_STABILIZER_AEROQUAD_H_


#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) // used only on mega for now

#include "CameraStabilizer.h"

// Written by CupOfTea:
// http://aeroquad.com/showthread.php?1484-Camera-Stablisation
// http://aeroquad.com/showthread.php?1491-Camera-Stablisation-continued

void initializeCameraControl() {
 // Init PWM Timer 1      Probable conflict with Arducopter Motor
 DDRB = DDRB | B11100000;                                  //Set to Output Mega Port-Pin PB5-11, PB6-12, PB7-13
                                                              //WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM 
 TCCR1A =((1<<WGM11)|(1<<COM1A1)|(1<<COM1B1)|(1<<COM1C1)); //Clear OCRnA/OCRnB/OCRnC outputs on compare match, set OCRnA/OCRnB/OCRnC outputs at BOTTOM (non-inverting mode).      
 TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);                 //Prescaler set to 8, that give us a resolution of 0.5us
 ICR1 = 39999;    //50hz freq (standard servos) 20ms = 40000 * 0.5us
}

void cameraControlMove() {
  if (mode > 0) {
    OCR1A = servoPitch * 2;
    OCR1B = servoRoll * 2;
    OCR1C = servoYaw * 2;
  }
}

#endif  // #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#endif  // #define _AEROQUAD_CAMERA_STABILIZER_H_
