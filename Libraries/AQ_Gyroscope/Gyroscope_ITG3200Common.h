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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_COMMON_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_COMMON_H_

#include <Gyroscope.h>

#ifdef ITG3200_ADDRESS_ALTERNATE
  #define ITG3200_ADDRESS					0x68
#else
  #define ITG3200_ADDRESS					0x69
#endif

#define GYRO_CALIBRATION_TRESHOLD 100

#define ITG3200_IDENTITY                0x68
#define ITG3200_IDENTITY_MASK           0x7E
#define ITG3200_MEMORY_ADDRESS			0x1D
#define ITG3200_BUFFER_SIZE				6
#define ITG3200_RESET_ADDRESS			0x3E
#define ITG3200_RESET_VALUE				0x80
#define ITG3200_LOW_PASS_FILTER_ADDR	0x16
#define ITG3200_LOW_PASS_FILTER_VALUE	0x1D	// 10Hz low pass filter
#define ITG3200_OSCILLATOR_ADDR			0x3E
#define ITG3200_OSCILLATOR_VALUE		0x01	// use X gyro oscillator
#define ITG3200_SCALE_TO_RADIANS		823.626831 // 14.375 LSBs per °/sec, / Pi / 180
#define ITG3200_TEMPERATURE_ADDRESS     0x1B


void measureGyro();
void measureSpecificGyroADC(long *gyroADC);
void measureSpecificGyroSum();
void evaluateSpecificGyroRate(long *gyroADC);

void initializeGyro() {
  if ((readWhoI2C(ITG3200_ADDRESS) & ITG3200_IDENTITY_MASK) == ITG3200_IDENTITY) {
	vehicleState |= GYRO_DETECTED;
  }
	
  gyroScaleFactor = radians(1.0 / 14.375);  //  ITG3200 14.375 LSBs per °/sec
  gyroOneMeterSecADCFactor = 1 / gyroScaleFactor;
  updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE); // send a reset to the device
  updateRegisterI2C(ITG3200_ADDRESS, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_VALUE); // 10Hz low pass filter
  updateRegisterI2C(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_OSCILLATOR_VALUE); // use internal oscillator 
}

void gyroUpdateHeading() {
  // Measure gyro heading
  unsigned long currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyro() {
  sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
  Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);

  measureSpecificGyroADC(gyroADC);
  
  for (byte axis = 0; axis <= ZAXIS; axis++) {
	gyroRate[axis] = gyroADC[axis] * gyroScaleFactor;
  }
 
  gyroUpdateHeading();
}

void measureGyroSum() {
  sendByteI2C(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS);
  Wire.requestFrom(ITG3200_ADDRESS, ITG3200_BUFFER_SIZE);
 
  measureSpecificGyroSum();
  gyroSampleCount++;
}

void evaluateGyroRate() {

  evaluateSpecificGyroRate(gyroADC);
  processGyroCommon();
}


boolean calibrateGyro() {

  for (int i=0; i < FINDZERO; i++) {
    measureGyroSum();
    delay(10);
  }
  for (byte axis = 0; axis < 3; axis++) {
    gyroZero[axis] = gyroSample[axis] / FINDZERO;
	gyroSample[axis] = 0;
	if (abs(gyroZero[axis]) >= GYRO_CALIBRATION_TRESHOLD) {
	  return false;
	}
  }
  gyroSampleCount = 0;
  return true;
}



#endif
