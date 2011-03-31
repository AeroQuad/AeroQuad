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

#include "Gyroscope_Wii.h"
#include <AQMath.h>
#include "Platform_Wii.h"

Gyroscope_Wii::Gyroscope_Wii() {
    // 0.5mV/º/s, 0.2mV/ADC step => 0.2/3.33 = around 0.069565217391304
    // ttp://invensense.com/mems/gyro/documents/PS-IDG-0650B-00-05.pdf and
    // http://invensense.com/mems/gyro/documents/ps-isz-0650b-00-05.pdf
  scaleFactor = radians(0.06201166); // Define the scale factor that converts to radians/second
}

void Gyroscope_Wii::initialize() {
}

void Gyroscope_Wii::measure() {
  // Replace code below with sensor measurement methodology
  for (byte axis = ROLL; axis < LASTAXIS; axis++) {
    gyroADC[axis] = getGyroADC(axis)  - zero[axis];
    rate[axis] = filterSmooth(gyroADC[axis] * scaleFactor, rate[axis], smoothFactor);
  }
}

void Gyroscope_Wii::calibrate() {
  // Add calibration method for measurement when gyro is motionless
  for (byte axis = ROLL; axis < LASTAXIS; axis++)
    zero[axis] = random(510, 514); // simulate zero measurement around 512
}

void Gyroscope_Wii::calibrate() {
  float findZero[FINDZERO];
    
  for (byte axis = 0; axis < 3; axis++) {
    for (int i=0; i<FINDZERO; i++) {
	  measure();
      findZero[i] = gyroADC[axis];
      delay(measureDelay);
    }
    zero[axis] = findMedian(findZero, FINDZERO);
  }
}