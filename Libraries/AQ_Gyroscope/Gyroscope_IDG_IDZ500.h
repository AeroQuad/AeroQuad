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

#ifndef _AEROQUAD_GYROSCOPE_IDG_IDZ500_H_
#define _AEROQUAD_GYROSCOPE_IDG_IDZ500_H_

#include <Gyroscope.h>
#include <SensorsStatus.h>

#define GYRO_CALIBRATION_TRESHOLD 4

#define AZPIN 12 // Auto zero pin for IDG500 gyros

int gyroChannel[3] = {0.0,0.0,0.0};
float gyroAref = 0.0;
  

void setGyroAref(float _aref) {
  gyroAref = _aref;
  gyroScaleFactor = radians((gyroAref/1024.0) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
} 

void initializeGyro() {
	vehicleState |= GYRO_DETECTED;
  analogReference(EXTERNAL);
  // Configure gyro auto zero pins
  pinMode (AZPIN, OUTPUT);
  digitalWrite(AZPIN, LOW);
  delay(1);

  // rollChannel = 4
  // pitchChannel = 3
  // yawChannel = 5
  gyroChannel[0] = 4;
  gyroChannel[1] = 3;
  gyroChannel[2] = 5;
}
    
void measureGyro() {
  int gyroADC;
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    if (axis == YAXIS)
      gyroADC = analogRead(gyroChannel[axis]) - gyroZero[axis];
    else
      gyroADC = gyroZero[axis] - analogRead(gyroChannel[axis]);
    gyroRate[axis] = gyroADC * gyroScaleFactor;
  }
 
  // Measure gyro heading
  long int currentTime = micros();
  if (gyroRate[ZAXIS] > radians(1.0) || gyroRate[ZAXIS] < radians(-1.0)) {
    gyroHeading += gyroRate[ZAXIS] * ((currentTime - gyroLastMesuredTime) / 1000000.0);
  }
  gyroLastMesuredTime = currentTime;
}

void measureGyroSum() {
  // do nothing here since it's already oversample in the APM_ADC class
}

void evaluateGyroRate() {
  // do nothing here since it's already oversample in the APM_ADC class
}

boolean calibrateGyro() {
  int findZero[FINDZERO];
  digitalWrite(AZPIN, HIGH);
  delayMicroseconds(750);
  digitalWrite(AZPIN, LOW);
  delay(8);

  int diff = 0;
  for (byte calAxis = XAXIS; calAxis <= ZAXIS; calAxis++) {
    for (int i=0; i<FINDZERO; i++) {
      findZero[i] = analogRead(gyroChannel[calAxis]);
	}
	int tmp = findMedianIntWithDiff(findZero, FINDZERO, &diff);
	if (diff <= GYRO_CALIBRATION_TRESHOLD) { // 4 = 0.27826087 degrees during 49*10ms measurements (490ms). 0.57deg/s difference between first and last.
	  gyroZero[calAxis] = tmp;
	} 
	else {
		return false; //Calibration failed.
	}
  }
  
  return true;
}

void readGyroTemp()  {
}

#endif
