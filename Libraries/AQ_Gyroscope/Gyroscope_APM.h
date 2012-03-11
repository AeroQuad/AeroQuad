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

#ifndef _AEROQUAD_GYROSCOPE_APM_H_
#define _AEROQUAD_GYROSCOPE_APM_H_

#include <Gyroscope.h>
#include <SensorsStatus.h>

void initializeGyro() {
  gyroScaleFactor = radians((3.3/4096) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
	vehicleState |= GYRO_DETECTED;
}
  
void measureGyro() {
  int gyroADC;
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    float rawADC = readADC(axis);
    if (rawADC > 500) // Check if good measurement
      if (axis == XAXIS)
        gyroADC =  rawADC - gyroZero[axis];
      else
        gyroADC =  gyroZero[axis] - rawADC;
    gyroRate[axis] = filterSmooth(gyroADC * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }
  
  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyroSum() {
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    gyroSample[axis] += readADC(axis);
  }
  gyroSampleCount++;
}

void evaluateGyroRate() {
  int gyroADC;
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    if (axis == XAXIS)
      gyroADC = (gyroSample[axis] / gyroSampleCount) - gyroZero[axis];
    else
      gyroADC = gyroZero[axis] - (gyroSample[axis] / gyroSampleCount);
    gyroRate[axis] = filterSmooth(gyroADC * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }
  gyroSample[0] = 0;
  gyroSample[1] = 0;
  gyroSample[2] = 0;
  gyroSampleCount = 0;
  
  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void calibrateGyro() {
  int findZero[FINDZERO];
   
  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
	  evaluateADC();
      findZero[i] = readADC(axis);
      delay(10);
    }
    gyroZero[axis] = findMedianInt(findZero, FINDZERO);
  }
}

#endif
