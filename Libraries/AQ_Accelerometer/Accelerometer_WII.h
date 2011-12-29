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

#ifndef _AEROQUAD_ACCELEROMETER_WII_H_
#define _AEROQUAD_ACCELEROMETER_WII_H_

#include <Accelerometer.h>
#include <Platform_Wii.h>

void initializeAccel() {
  vehicleState |= ACCEL_DETECTED;
}

void measureAccel() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSec[axis] = getWiiAccelADC(axis) * accelScaleFactor[axis] + runTimeAccelBias[axis];
  }
}

void measureAccelSum() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
	accelSample[axis] += getWiiAccelADC(axis);
  }
  accelSampleCount++;
}

void evaluateMetersPerSec() {
  // do nothing here
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    readWiiSensors();
    measureAccelSum();
  }

  for (byte axis = 0; axis <= ZAXIS; axis++) {
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
