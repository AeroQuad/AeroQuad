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

#ifndef _AEROQUAD_ACCELEROMETER_WII_H_
#define _AEROQUAD_ACCELEROMETER_WII_H_

#include <Accelerometer.h>
#include <Platform_Wii.h>
#include <SensorsStatus.h>

void initializeAccel() {
  vehicleState |= ACCEL_DETECTED;
}

void measureAccel() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = getWiiAccelADC(axis) * accelScaleFactor[axis] + runTimeAccelBias[axis];
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
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = fabs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

#endif
