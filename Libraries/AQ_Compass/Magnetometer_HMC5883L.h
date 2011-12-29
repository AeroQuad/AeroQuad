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


#ifndef _AEROQUAD_MAGNETOMETER_HMC5883L_H_
#define _AEROQUAD_MAGNETOMETER_HMC5883L_H_

#include "Compass.h"

#include "Arduino.h"

#define COMPASS_ADDRESS 0x1E

// See HMC58x3 datasheet for more information on these values
#define NormalOperation             0x10
// Default DataOutputRate is 10hz on HMC5843 , 15hz on HMC5883L
#define DataOutputRate_Default      ( 0x04 << 2 )
#define HMC5883L_SampleAveraging_8  ( 0x03 << 5 )


float magCalibration[3] = {0.0,0.0,0.0};
  
void initializeMagnetometer() {

  byte numAttempts = 0;
  bool success = false;
  delay(10);                             // Power up delay **
   
  magCalibration[XAXIS] = 1.0;
  magCalibration[YAXIS] = 1.0;
  magCalibration[ZAXIS] = 1.0;

  if (readWhoI2C(COMPASS_ADDRESS) != COMPASS_ADDRESS) {
	  vehicleState |= MAG_DETECTED;
  }    

  while (success == false && numAttempts < 5 ) {
     
    numAttempts++;
  
    updateRegisterI2C(COMPASS_ADDRESS, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
    delay(50);
    updateRegisterI2C(COMPASS_ADDRESS, 0x01, 0x20); // Set +/- 1G gain
    delay(10);
    updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01);  // Perform single conversion
    delay(10);
   
    measureMagnetometer(0.0, 0.0);                    // Read calibration data
    delay(10);
   
    if ( fabs(measuredMagX) > 500.0 && fabs(measuredMagX) < 1564.4f 
        && fabs(measuredMagY) > 500.0 && fabs(measuredMagY) < 1564.4f 
        && fabs(measuredMagZ) > 500.0 && fabs(measuredMagZ) < 1477.2f) {
      magCalibration[XAXIS] = fabs(1264.4f / measuredMagX);
      magCalibration[YAXIS] = fabs(1264.4f / measuredMagY);
      magCalibration[ZAXIS] = fabs(1177.2f / measuredMagZ); 
      success = true;
    }
   
    updateRegisterI2C(COMPASS_ADDRESS, 0x00, HMC5883L_SampleAveraging_8 | DataOutputRate_Default | NormalOperation);
    delay(50);

    updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x00); // Continuous Update mode
    delay(50);                           // Mode change delay (1/Update Rate) **
  }

  measureMagnetometer(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
  
}

void measureMagnetometer(float roll, float pitch) {
   
  sendByteI2C(COMPASS_ADDRESS, 0x03);
  Wire.requestFrom(COMPASS_ADDRESS, 6);

  #if defined(SPARKFUN_9DOF)
    // JI - 11/24/11 - SparkFun DOF on v2p1 Shield Configuration
    // JI - 11/24/11 - 5883L X axis points aft
    // JI - 11/24/11 - 5883L Sensor Orientation 3
    rawMag[XAXIS] = -readShortI2C();
	rawMag[ZAXIS] = -readShortI2C();
    rawMag[YAXIS] =  readShortI2C();
  #elif defined(SPARKFUN_5883L_BOB)
    // JI - 11/24/11 - Sparkfun 5883L Breakout Board Upside Down on v2p0 shield
    // JI - 11/24/11 - 5883L is upside down, X axis points forward
    // JI - 11/24/11 - 5883L Sensor Orientation 5
    rawMag[XAXIS] = readShortI2C();
	rawMag[ZAXIS] = readShortI2C();
    rawMag[YAXIS] = readShortI2C();
  #else
    //!! Define 5883L Orientation !!
  #endif
  
  measuredMagX = rawMag[XAXIS] + magBias[XAXIS];
  measuredMagY = rawMag[YAXIS] + magBias[YAXIS];
  measuredMagZ = rawMag[ZAXIS] + magBias[ZAXIS];


  float cosRoll =  cos(roll);
  float sinRoll =  sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  float magX = (float)measuredMagX * cosPitch + 
         (float)measuredMagY * sinRoll * sinPitch + 
         (float)measuredMagZ * cosRoll * sinPitch;
           
  float magY = (float)measuredMagY * cosRoll - 
         (float)measuredMagZ * sinRoll;
		 
  float tmp = sqrt(magX * magX + magY * magY);
    
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

#endif