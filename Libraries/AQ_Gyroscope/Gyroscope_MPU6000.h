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

#ifndef _AEROQUAD_GYROSCOPE_MPU6000_COMMON_H_
#define _AEROQUAD_GYROSCOPE_MPU6000_COMMON_H_

int gyroRaw[3] = {0,0,0};

#include <Platform_MPU6000.h>
#include <Gyroscope.h>

void initializeGyro() {
  float range = 2*1000.0;
  gyroScaleFactor = radians(range/65536.0);

  initializeMPU6000Sensors();
}

void GyroUpdateHeading()
{
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > (float)radians(1.0) || gyroRate[ZAXIS] < (float)radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyro() {
  readMPU6000Gyro();

  int gyroADC[3];
  gyroADC[XAXIS] = (gyroRaw[XAXIS]=MPU6000.data.gyro.x)  - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroRaw[YAXIS]=MPU6000.data.gyro.y);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroRaw[ZAXIS]=MPU6000.data.gyro.z);

  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }

  GyroUpdateHeading();
}

void measureGyroSum() {
  readMPU6000Gyro();
  gyroSample[XAXIS] += (gyroRaw[XAXIS]=MPU6000.data.gyro.x);
  gyroSample[YAXIS] += (gyroRaw[YAXIS]=MPU6000.data.gyro.y);
  gyroSample[ZAXIS] += (gyroRaw[ZAXIS]=MPU6000.data.gyro.z);

  gyroSampleCount++;
}

void evaluateGyroRate() {
  int gyroADC[3];
  gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroSample[YAXIS] / gyroSampleCount);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);

  gyroSample[XAXIS] = 0;
  gyroSample[YAXIS] = 0;
  gyroSample[ZAXIS] = 0;
  gyroSampleCount = 0;

  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }

  GyroUpdateHeading();
}


void calibrateGyro() {
  int findZero[FINDZERO];

  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      readMPU6000Sensors();
      short data;
      if(axis == XAXIS) {
    	  data = MPU6000.data.gyro.x;
      } else if(axis == YAXIS) {
    	  data = MPU6000.data.gyro.y;
      } else {
    	  data = MPU6000.data.gyro.z;
      }
      findZero[i] = data;
      delay(10);
    }
    gyroZero[axis] = findMedianInt(findZero, FINDZERO);
  }
}

#endif
