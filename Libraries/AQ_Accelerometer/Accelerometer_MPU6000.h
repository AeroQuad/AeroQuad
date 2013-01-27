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

#ifndef _AEROQUAD_ACCELEROMETER_MPU6000_COMMON_H_
#define _AEROQUAD_ACCELEROMETER_MPU6000_COMMON_H_

#include <Platform_MPU6000.h>
#include <Accelerometer.h>

void initializeAccel() {
  initializeMPU6000Sensors();
}


void measureAccel() {
  readMPU6000Accel();

  meterPerSecSec[XAXIS] = MPU6000.data.accel.x * accelScaleFactor[XAXIS] + runTimeAccelBias[XAXIS];
  meterPerSecSec[YAXIS] = MPU6000.data.accel.y * accelScaleFactor[YAXIS] + runTimeAccelBias[YAXIS];
  meterPerSecSec[ZAXIS] = MPU6000.data.accel.z * accelScaleFactor[ZAXIS] + runTimeAccelBias[ZAXIS];
}

void measureAccelSum() {
  readMPU6000Accel();
  accelSample[XAXIS] += MPU6000.data.accel.x;
  accelSample[YAXIS] += MPU6000.data.accel.y;
  accelSample[ZAXIS] += MPU6000.data.accel.z;

  accelSampleCount++;
}

void evaluateMetersPerSec() {
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
  	accelSample[axis] = 0;
  }
  accelSampleCount = 0;
}

void computeAccelBias() {
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
	readMPU6000Sensors();
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[XAXIS] = -meterPerSecSec[XAXIS];
  runTimeAccelBias[YAXIS] = -meterPerSecSec[YAXIS];
  runTimeAccelBias[ZAXIS] = -9.8065 - meterPerSecSec[ZAXIS];

  accelOneG = abs(meterPerSecSec[ZAXIS] + runTimeAccelBias[ZAXIS]);
}

#endif
