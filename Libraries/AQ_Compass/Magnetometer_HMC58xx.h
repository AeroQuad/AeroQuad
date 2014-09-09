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


#ifndef _AEROQUAD_MAGNETOMETER_HMC58XX_H_
#define _AEROQUAD_MAGNETOMETER_HMC58XX_H_

#include "Compass.h"
#include <SensorsStatus.h>

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

void readSpecificMag(float *rawMag);


void initializeMagnetometer() {
  delay(10);                             // Power up delay **
   
  sendByteI2C(COMPASS_ADDRESS, 10);
  Wire.requestFrom(COMPASS_ADDRESS, 3);
  if(Wire.available() == 3) {
    byte id1 = Wire.read();
    byte id2 = Wire.read();
    byte id3 = Wire.read();
    if(id1 == 'H' && id2 == '4' && id3 == '3') {
      vehicleState |= MAG_DETECTED;
      
	  updateRegisterI2C(COMPASS_ADDRESS, 0x01, SENSOR_GAIN); // Gain as defined above
      delay(20);

      updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01); // start single conversion
      delay(20);

      measureMagnetometer(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
    }
  }  
}

void measureMagnetometer(float roll, float pitch) {
    
  sendByteI2C(COMPASS_ADDRESS, 0x03);
  Wire.requestFrom(COMPASS_ADDRESS, 6);

  readSpecificMag(rawMag);

  updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01); // start single conversion

  measuredMag[XAXIS] = rawMag[XAXIS] + magBias[XAXIS];
  measuredMag[YAXIS] = rawMag[YAXIS] + magBias[YAXIS];
  measuredMag[ZAXIS] = rawMag[ZAXIS] + magBias[ZAXIS];
  
  const float cosRoll =  cos(roll);
  const float sinRoll =  sin(roll);
  const float cosPitch = cos(pitch);
  const float sinPitch = sin(pitch);

  const float magX = measuredMag[XAXIS] * cosPitch + 
                     measuredMag[YAXIS] * sinRoll * sinPitch + 
                     measuredMag[ZAXIS] * cosRoll * sinPitch;
           
  const float magY = measuredMag[YAXIS] * cosRoll - 
                     measuredMag[ZAXIS] * sinRoll;

  const float tmp  = sqrt(magX * magX + magY * magY);
   
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

#endif
