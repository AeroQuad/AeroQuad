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

#ifndef _AEROQUAD_GYROSCOPE_APM_H_
#define _AEROQUAD_GYROSCOPE_APM_H_

#include <Gyroscope.h>

//#define APM_SCALE_TO_RADIANS radians((3.3/4096) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)

class Gyroscope_APM : public Gyroscope {
public:
  Gyroscope_APM() {
    gyroScaleFactor = radians((3.3/4096) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
    smoothFactor = 1.0;
  }
  
  void measure(void) {
    int gyroADC;
    for (byte axis = ROLL; axis <= YAW; axis++) {
      float rawADC = readADC(axis);
      if (rawADC > 500) // Check if good measurement
        if (axis == ROLL)
          gyroADC =  rawADC - zero[axis];
        else
          gyroADC =  zero[axis] - rawADC;
      rate[axis] = filterSmooth(gyroADC * gyroScaleFactor, rate[axis], smoothFactor);
    }
  
    // Measure gyro heading
    long int currentTime = micros();
    if (rate[YAW] > radians(1.0) || rate[YAW] < radians(-1.0)) {
      heading += rate[YAW] * ((currentTime - lastMesuredTime) / 1000000.0);
    }
    lastMesuredTime = currentTime;
  }

  void calibrate() {
    int findZero[FINDZERO];
    
    for (byte axis = 0; axis < 3; axis++) {
      for (int i=0; i<FINDZERO; i++) {
	    findZero[i] = readADC(axis);
        delay(10);
      }
      zero[axis] = findMedianInt(findZero, FINDZERO);
    }
  }
};
#endif
