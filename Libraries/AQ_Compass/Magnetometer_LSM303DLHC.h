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


#ifndef _AEROQUAD_MAGNETOMETER_LSM303DLHC_H_
#define _AEROQUAD_MAGNETOMETER_LSM303DLHC_H_

#include "Compass.h"
#include <SensorsStatus.h>

#include "Arduino.h"

#define COMPASS_ADDRESS 0x1E
#define COMPASS_IDENTITY 0x10

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
  delay(10);
  vehicleState |= MAG_DETECTED;  //for now we just assume its working

/*
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x00);
  Wire.write(0b00010000);   // data output rate
  Wire.endTransmission();  
*/

  // Enable Magnetometer  
  

  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x02);
  Wire.write(0x00);   // Continuous conversion mode  
  Wire.endTransmission();     

  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x01);
  Wire.write(0b00100000);   // +/- 1.3 Gauss gain //page 37
  Wire.endTransmission();  

  measureMagnetometer(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
}

void measureMagnetometer(float roll, float pitch) {
    
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(COMPASS_ADDRESS, 6);
  byte xhm, xlm, zhm, zlm, yhm, ylm;
  xhm = Wire.read();
  xlm = Wire.read();
  zhm = Wire.read();
  zlm = Wire.read();
  yhm = Wire.read();
  ylm = Wire.read();

  // combine high and low bytes
  int compassx = (int16_t)(xhm << 8 | xlm);
  int compassy = (int16_t)(yhm << 8 | ylm);
  int compassz = (int16_t)(zhm << 8 | zlm);

  rawMag[XAXIS] = compassx;
  rawMag[YAXIS] = compassy;
  rawMag[ZAXIS] = compassz;

  measuredMagX = rawMag[XAXIS] + magBias[XAXIS];
  measuredMagY = rawMag[YAXIS] + magBias[YAXIS];
  measuredMagZ = rawMag[ZAXIS] + magBias[ZAXIS];
  
  measuredMag[XAXIS] = measuredMagX;
  measuredMag[YAXIS] = measuredMagY;
  measuredMag[ZAXIS] = measuredMagZ;
  
  float cosRoll =  cos(roll);
  float sinRoll =  sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  float magX = (float)measuredMagX * cosPitch + (float)measuredMagY * sinRoll * sinPitch + (float)measuredMagZ * cosRoll * sinPitch;
  float magY = (float)measuredMagY * cosRoll - (float)measuredMagZ * sinRoll;
  float tmp  = sqrt(magX * magX + magY * magY);
   
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

#endif
