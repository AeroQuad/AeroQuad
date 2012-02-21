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

#ifndef _AEROQUAD_DEVICE_I2C_H_
#define _AEROQUAD_DEVICE_I2C_H_

// I2C functions
#include "Arduino.h"
#include <Wire.h>

void sendByteI2C(int deviceAddress, byte dataValue);
byte readByteI2C(int deviceAddress);
int readWordI2C(int deviceAddress);
int readWordI2C();
int readShortI2C(int deviceAddress);
int readShortI2C();
int readReverseShortI2C();
int readWordWaitI2C(int deviceAddress);
int readReverseWordI2C(int deviceAddress);
byte readWhoI2C(int deviceAddress);
void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue);

#endif





