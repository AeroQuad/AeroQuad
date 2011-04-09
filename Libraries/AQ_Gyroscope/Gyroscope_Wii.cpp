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

Gyroscope_Wii::Gyroscope_Wii() {
    // 0.5mV/º/s, 0.2mV/ADC step => 0.2/3.33 = around 0.069565217391304
    // ttp://invensense.com/mems/gyro/documents/PS-IDG-0650B-00-05.pdf and
    // http://invensense.com/mems/gyro/documents/ps-isz-0650b-00-05.pdf
  scaleFactor = radians(0.06201166); // Define the scale factor that converts to radians/second
}

void Gyroscope_Wii::measure() {
  // Replace code below with sensor measurement methodology
  wii.measure();
  for (byte axis = ROLL; axis <= YAW; axis++) {
    gyroADC[axis] = wii.getGyroADC(axis)  - zero[axis];
    rate[axis] = filterSmooth(gyroADC[axis] * scaleFactor, rate[axis], smoothFactor);
  }
  
  // Measure gyro heading
  long int currentTime = micros();
  if (rate[YAW] > radians(1.0) || rate[YAW] < radians(-1.0)) {
    heading += rate[YAW] * ((currentTime - lastMesuredTime) / 1000000.0);
  }
  lastMesuredTime = currentTime;

}

void Gyroscope_Wii::calibrate() {
  int findZero[FINDZERO];
    
  for (byte axis = ROLL; axis <= YAW; axis++) {
    for (int i=0; i<FINDZERO; i++) {
	  wii.measure();
      findZero[i] = wii.getGyroADC(axis);
      delay(5);
    }
    zero[axis] = findMedianInt(findZero, FINDZERO);
  }
}