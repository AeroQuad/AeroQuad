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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_9DOF_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_9DOF_H_

#include <SensorsStatus.h>
#include <Gyroscope_ITG3200Common.h>

void measureSpecificGyroADC(int *gyroADC) {
  gyroADC[YAXIS] = readShortI2C()  - gyroZero[YAXIS];
  gyroADC[XAXIS] = readShortI2C()  - gyroZero[XAXIS];
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - readShortI2C();
}

void measureSpecificGyroSum() {
  gyroSample[YAXIS] += readShortI2C();
  gyroSample[XAXIS] += readShortI2C();
  gyroSample[ZAXIS] += readShortI2C();
}

void evaluateSpecificGyroRate(int *gyroADC) {

  gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
  gyroADC[YAXIS] = (gyroSample[YAXIS] / gyroSampleCount) - gyroZero[YAXIS];
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);
}

boolean calibrateGyro() {
  //Finds gyro drift.
  //Returns false if during calibration there was movement of board. 

  int findZero[FINDZERO];
  int diff = 0;

  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
      sendByteI2C(ITG3200_ADDRESS, (axis * 2) + ITG3200_MEMORY_ADDRESS);
      findZero[i] = readShortI2C(ITG3200_ADDRESS);
      delay(10);
    }

	if (axis == XAXIS) {
	  int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	  if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
	    gyroZero[YAXIS] = tmp;
  	  } 
	  else {
	    return false; //Calibration failed.
	  }
    }
    else if (axis == YAXIS) {
	  int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	  if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
	    gyroZero[XAXIS] = tmp;
  	  } 
	  else {
	    return false; //Calibration failed.
	  }
    }
    else {
	  int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	  if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
	    gyroZero[ZAXIS] = tmp;
  	  } 
	  else {
		return false; //Calibration failed.
	  }
    }
  }

  return true; //Calibration successfull.
}

#endif
