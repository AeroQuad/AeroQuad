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

#ifndef _AQ_BAROMETRIC_SENSOR_
#define _AQ_BAROMETRIC_SENSOR_

#include "Arduino.h"
#include "GlobalDefined.h"

float baroAltitude      = 0.0; 
float baroRawAltitude   = 0.0;
float baroGroundAltitude = 0.0;
float baroSmoothFactor   = 0.02;
  
// **********************************************************************
// The following function calls must be defined inside any new subclasses
// **********************************************************************
void initializeBaro(); 
void measureBaro();
void measureBaroSum();
void evaluateBaroAltitude();
  
// *********************************************************
// The following functions are common between all subclasses
// *********************************************************
const float getBaroAltitude() {
  return baroAltitude - baroGroundAltitude;
}
 
void measureGroundBaro() {
  // measure initial ground pressure (multiple samples)
  float altSum = 0.0;
  for (int i=0; i < 25; i++) {
    measureBaro();
	altSum += baroRawAltitude;
    delay(12);
  }
  baroGroundAltitude = altSum / 25;
}




#endif