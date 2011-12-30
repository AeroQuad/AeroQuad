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


#ifndef _AEROQUAD_MAGNETOMETER_HMC5843_H_
#define _AEROQUAD_MAGNETOMETER_HMC5843_H_

#include "Compass.h"

#include "Arduino.h"

#define COMPASS_ADDRESS 0x1E
//#define SENSOR_GAIN 0x00  // +/- 0.7 Ga
#define SENSOR_GAIN 0x20  // +/- 1.0 Ga (default)
//#define SENSOR_GAIN 0x40  // +/- 1.5 Ga
//#define SENSOR_GAIN 0x60  // +/- 2.0 Ga
//#define SENSOR_GAIN 0x80  // +/- 3.2 Ga
//#define SENSOR_GAIN 0xA0  // +/- 3.8 Ga
//#define SENSOR_GAIN 0xC0  // +/- 4.5 Ga
//#define SENSOR_GAIN 0xE0  // +/- 6.5 Ga (not recommended)

void initializeMagnetometer() {

  delay(10);                             // Power up delay **
   
  if (readWhoI2C(COMPASS_ADDRESS) != COMPASS_ADDRESS) {
	  vehicleState |= MAG_DETECTED;
  }    

  updateRegisterI2C(COMPASS_ADDRESS, 0x01, SENSOR_GAIN); // Gain as defined above
  delay(20);
  updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01); // start single conversion
  delay(20);

  measureMagnetometer(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
}

void measureMagnetometer(float roll, float pitch) {
  float magX;
  float magY;
  float tmp;
    
  sendByteI2C(COMPASS_ADDRESS, 0x03);
  Wire.requestFrom(COMPASS_ADDRESS, 6);

  rawMag[XAXIS] =  readShortI2C();
  rawMag[YAXIS] = -readShortI2C();
  rawMag[ZAXIS] = -readShortI2C();

  updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01); // start single conversion

  measuredMagX = rawMag[XAXIS] + magBias[XAXIS];
  measuredMagY = rawMag[YAXIS] + magBias[YAXIS];
  measuredMagZ = rawMag[ZAXIS] + magBias[ZAXIS];

  float cosRoll =  cos(roll);
  float sinRoll =  sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  magX = (float)measuredMagX * cosPitch + 
         (float)measuredMagY * sinRoll * sinPitch + 
         (float)measuredMagZ * cosRoll * sinPitch;
           
  magY = (float)measuredMagY * cosRoll - 
         (float)measuredMagZ * sinRoll;

  tmp  = sqrt(magX * magX + magY * magY);
   
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

#endif