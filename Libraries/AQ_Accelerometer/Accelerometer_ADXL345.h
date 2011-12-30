/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AEROQUAD_ACCELEROMETER_ADXL345_H_
#define _AEROQUAD_ACCELEROMETER_ADXL345_H_

#include <Accelerometer.h>
#include <SensorsStatus.h>

#define ACCEL_ADDRESS 0x53

void initializeAccel() {

  if (readWhoI2C(ACCEL_ADDRESS) !=  0xE5) { 			// page 14 of datasheet
    vehicleState |= ACCEL_DETECTED;
  }
	
  updateRegisterI2C(ACCEL_ADDRESS, 0x2D, 1<<3); 	// set device to *measure*
  updateRegisterI2C(ACCEL_ADDRESS, 0x31, 0x08); 	// set full range and +/- 2G
  updateRegisterI2C(ACCEL_ADDRESS, 0x2C, 8+2+1);    // 200hz sampling
  delay(10); 
}
  
void measureAccel() {

  sendByteI2C(ACCEL_ADDRESS, 0x32);
  Wire.requestFrom(ACCEL_ADDRESS, 6);

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSec[axis] = readReverseShortI2C() * accelScaleFactor[axis] + runTimeAccelBias[axis];
  }
}

void measureAccelSum() {

  sendByteI2C(ACCEL_ADDRESS, 0x32);
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    accelSample[axis] += readReverseShortI2C();
  }
  accelSampleCount++;
}

void evaluateMetersPerSec() {
	
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  accelSampleCount = 0;		
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSec[ZAXIS];

  accelOneG = abs(meterPerSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}



#endif
