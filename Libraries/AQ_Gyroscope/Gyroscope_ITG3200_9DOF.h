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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_9DOF_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_9DOF_H_

#include <SensorsStatus.h>
#include <Gyroscope_ITG3200Common.h>

void measureSpecificGyroADC(int *gyroADC) {
  gyroADC[YAXIS] = readShortI2C() - gyroZero[YAXIS];
  gyroADC[XAXIS] = readShortI2C() - gyroZero[XAXIS];
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

void calibrateGyro() {

  int findZero[FINDZERO];
    for (byte calAxis = XAXIS; calAxis <= ZAXIS; calAxis++) {
      for (int i=0; i<FINDZERO; i++) {
        sendByteI2C(ITG3200_ADDRESS, (calAxis * 2) + 0x1D);
        findZero[i] = readShortI2C(ITG3200_ADDRESS);
        delay(10);
      }
      if (calAxis == XAXIS) {
        gyroZero[YAXIS] = findMedianInt(findZero, FINDZERO);
      }
      else if (calAxis == YAXIS) {
        gyroZero[XAXIS] = findMedianInt(findZero, FINDZERO);
      }
      else {
        gyroZero[ZAXIS] = findMedianInt(findZero, FINDZERO);
      }
    }
}

#endif
