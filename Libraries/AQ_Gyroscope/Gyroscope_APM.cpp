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

#include "Gyroscope_APM.h"

#include <APM_ADC.h>
#include <AQMath.h>


Gyroscope_APM::Gyroscope_APM() {
  gyroScaleFactor = radians((3.3/4096) / 0.002);  // IDG/IXZ500 sensitivity = 2mV/(deg/sec)
  measureDelay = 2;	// process or reading time for ITG3200 is 2ms
}
  
void Gyroscope_APM::initialize(void) {
  smoothFactor = 1.0;
}
  
void Gyroscope_APM::measure(void) {
  unsigned long currentTime = millis();
  if ((currentTime - lastMeasuredTime) >= measureDelay) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      float rawADC = readADC(axis);
      if (rawADC > 500) // Check if good measurement
        if (axis == ROLL)
          gyroADC[axis] =  rawADC - gyroZero[axis];
        else
          gyroADC[axis] =  gyroZero[axis] - rawADC;
      gyroData[axis] = filterSmooth(gyroADC[axis] * gyroScaleFactor, gyroData[axis], smoothFactor);
    }
	lastMeasuredTime = currentTime;
  }
}

void Gyroscope_APM::calibrate() {
  int findZero[FINDZERO];
    
  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
	  measure();
      findZero[i] = gyroADC[axis];
      delay(measureDelay);
    }
    zero[axis] = findMedianInt(findZero, FINDZERO);
  }
}