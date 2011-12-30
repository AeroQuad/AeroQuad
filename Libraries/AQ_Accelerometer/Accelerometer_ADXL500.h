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

#ifndef _AEROQUAD_ACCELEROMETER_IDG500_H_
#define _AEROQUAD_ACCELEROMETER_IDG500_H_

#include <Accelerometer.h>

void initializeAccel() {
  vehicleState |= ACCEL_DETECTED;
}

void measureAccel() {

  meterPerSec[XAXIS] = analogRead(1) * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSec[YAXIS] = analogRead(0) * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSec[ZAXIS] = analogRead(2) * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];
}

void measureAccelSum() {
  
  accelSample[XAXIS] += analogRead(1);
  accelSample[YAXIS] += analogRead(0);
  accelSample[ZAXIS] += analogRead(2);
  accelSampleCount++;
}

void evaluateMetersPerSec() {
  // do nothing here
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
