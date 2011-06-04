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

class Accelerometer_ADXL500 : public Accelerometer {
private:
  float aref;
  
public:
  Accelerometer_ADXL500() {
    accelScaleFactor = G_2_MPS2((3.0/1024.0) / 0.300);  // force aref to 3.0 for v1.7 shield
    smoothFactor     = 1.0;
  }

  void setAref(float aref) {
	this->aref = aref;
	accelScaleFactor = G_2_MPS2((aref/1024.0) / 0.300);
  }
  
  void measure(void) {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2
  
    int accelADC[3];
    accelADC[XAXIS] = analogRead(1) - zero[PITCH];
    accelADC[YAXIS] = analogRead(0) - zero[ROLL];
    accelADC[ZAXIS] = zero[ZAXIS] - analogRead(2);
    for (byte axis = XAXIS; axis < LASTAXIS; axis++) {
      meterPerSec[axis] = filterSmooth(accelADC[axis] * accelScaleFactor, meterPerSec[axis], smoothFactor);
    }
  }

  void calibrate() {
    // rollChannel = 1
    // pitchChannel = 0
    // zAxisChannel = 2

    int findZero[FINDZERO];

    for (int i=0; i<FINDZERO; i++) {
      findZero[i] = analogRead(1);
	  delay(2);
    }
    zero[XAXIS] = findMedianInt(findZero, FINDZERO);
    for (int i=0; i<FINDZERO; i++) {
	  findZero[i] = analogRead(0);
	  delay(2);
    }
    zero[YAXIS] = findMedianInt(findZero, FINDZERO);
    for (int i=0; i<FINDZERO; i++) {
	  findZero[i] = analogRead(2);
	  delay(2);
    }
    zero[ZAXIS] = findMedianInt(findZero, FINDZERO);
	
    // store accel value that represents 1g
    measure();
    oneG = -meterPerSec[ZAXIS];
    // replace with estimated Z axis 0g value
    zero[ZAXIS] = (zero[XAXIS] + zero[YAXIS]) / 2;
  }

};
#endif
