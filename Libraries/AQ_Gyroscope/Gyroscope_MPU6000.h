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

#define GYRO_CALIBRATION_TRESHOLD 60

void initializeGyro() {
  float range = 2 * 2000.0;
  gyroScaleFactor = radians(range/65536.0);

  gyroOneMeterSecADCFactor = 1 / gyroScaleFactor;
	
  initializeMPU6000Sensors();
}

void gyroUpdateHeading()
{
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > (float)radians(1.0) || gyroRate[ZAXIS] < (float)radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyro() {
  readMPU6000Sensors();

  gyroADC[XAXIS] = (gyroRaw[XAXIS] = MPU6000.data.gyro.x) - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroRaw[YAXIS] = MPU6000.data.gyro.y);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroRaw[ZAXIS] = MPU6000.data.gyro.z);

  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = gyroADC[axis] * gyroScaleFactor;
  }

  gyroUpdateHeading();
}

void measureGyroSum() {

  gyroSample[XAXIS] += (gyroRaw[XAXIS]=MPU6000.data.gyro.x);
  gyroSample[YAXIS] += (gyroRaw[YAXIS]=MPU6000.data.gyro.y);
  gyroSample[ZAXIS] += (gyroRaw[ZAXIS]=MPU6000.data.gyro.z);
  
  gyroSampleCount++;
}

void evaluateGyroRate() {

  gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroSample[YAXIS] / gyroSampleCount);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);

  processGyroCommon();
}




boolean calibrateGyro() {
  
  long findZero[3] = {0,0,0};
  for (int i=0; i < FINDZERO; i++) {
    readMPU6000Sensors();
    findZero[XAXIS] += MPU6000.data.gyro.x;
    findZero[YAXIS] += MPU6000.data.gyro.y;
    findZero[ZAXIS] += MPU6000.data.gyro.z;
    delay(10);
  }

  for (byte axis = 0; axis < 3; axis++) {
    gyroZero[axis] = findZero[axis] / FINDZERO;
	if (abs(gyroZero[axis]) >= GYRO_CALIBRATION_TRESHOLD) {
	  return false;
	}
  }
  return true;
}

#endif
