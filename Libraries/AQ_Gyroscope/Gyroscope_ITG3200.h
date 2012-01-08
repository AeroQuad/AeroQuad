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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_H_

#include <Gyroscope.h>
#include <SensorsStatus.h>
#include <Gyroscope_ITG3200Defines.h>

#ifndef ITG3200_ADDRESS_ALTERNATE
  #define ITG3200_ADDRESS					0x69
#else
  #define ITG3200_ADDRESS					0x68
#endif
#define ITG3200_IDENTITY 0x69

void initializeGyro() {

  if (readWhoI2C(ITG3200_ADDRESS) == ITG3200_IDENTITY) {
	  vehicleState |= GYRO_DETECTED;
  }
	
  gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
  updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE); // send a reset to the device
  updateRegisterI2C(ITG3200_ADDRESS, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_VALUE); // 10Hz low pass filter
  updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_OSCILLATOR_VALUE); // use internal oscillator 
}
    
void measureGyro() {
  sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
  Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);
    
  // The following 3 lines read the gyro and assign it's data to gyroADC
  // in the correct order and phase to suit the standard shield installation
  // orientation.  See TBD for details.  If your shield is not installed in this
  // orientation, this is where you make the changes.
  int gyroADC[3];
  gyroADC[XAXIS] = readShortI2C()  - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - readShortI2C();
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - readShortI2C();

  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }
 
  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyroSum() {
  sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
  Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    gyroSample[axis] += readShortI2C();
  }
  gyroSampleCount++;
}

void evaluateGyroRate() {
  int gyroADC[3];
  gyroADC[XAXIS] = (gyroSample[XAXIS] / gyroSampleCount) - gyroZero[XAXIS];
  gyroADC[YAXIS] = gyroZero[YAXIS] - (gyroSample[YAXIS] / gyroSampleCount);
  gyroADC[ZAXIS] = gyroZero[ZAXIS] - (gyroSample[ZAXIS] / gyroSampleCount);
  gyroSample[0] = 0;
  gyroSample[1] = 0;
  gyroSample[2] = 0;
  gyroSampleCount = 0;

  for (byte axis = 0; axis <= ZAXIS; axis++) {
    gyroRate[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroRate[axis], gyroSmoothFactor);
  }
 
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
      sendByteI2C(ITG3200_ADDRESS, (axis * 2) + ITG3200_LOW_PASS_FILTER_VALUE);
      findZero[i] = readShortI2C(ITG3200_ADDRESS);
      delay(10);
    }
    gyroZero[axis] = findMedianInt(findZero, FINDZERO);
  }
}

#endif
