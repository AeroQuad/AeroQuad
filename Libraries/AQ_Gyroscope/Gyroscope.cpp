/*
  AeroQuad v3.0 - February 2011
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

#include "Gyroscope.h"

#include "AQMath.h"

Gyroscope::Gyroscope() {
}

void Gyroscope::setZero(byte axis,float value) {
  zero[axis] = value;
}

const float Gyroscope::getZero(byte axis) {
  return zero[axis];
}
  
const float Gyroscope::getSmoothFactor() {
  return smoothFactor;
}

void Gyroscope::setSmoothFactor(float value) {
  smoothFactor = value;
}

const float Gyroscope::getRadPerSec(byte axis) {
  return rate[axis];
}

void Gyroscope::calibrate() {
  float findZero[FINDZERO];
    
  for (byte calAxis = 0; calAxis < 3; calAxis++) {
    for (int i=0; i<FINDZERO; i++) {
	  measure();
      findZero[i] = getRadPerSec(calAxis);
      delay(measureDelay);
    }
    zero[calAxis] = findMedian(findZero, FINDZERO);
  }
}
