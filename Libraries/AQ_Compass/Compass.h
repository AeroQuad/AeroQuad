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


#ifndef _AEROQUAD_COMPASS_H_
#define _AEROQUAD_COMPASS_H_

#include "Arduino.h"

float hdgX = 0.0;
float hdgY = 0.0;

float measuredMagX = 0.0;
float measuredMagY = 0.0;
float measuredMagZ = 0.0;
float measuredMag[3] = {0.0,0.0,0.0};
float rawMag[3] = {0.0,0.0,0.0};
float magBias[3] = {0.0,0.0,0.0};

void initializeMagnetometer();
void measureMagnetometer(float roll, float pitch);

const float getHdgXY(byte axis) {
  if (axis == XAXIS) {
    return hdgX;
  } else {
    return hdgY;
  }
}

const int getMagnetometerRawData(byte axis) {
  return rawMag[axis];
}

const int getMagnetometerData(byte axis) {
  return measuredMag[axis];
}


const float getAbsoluteHeading() {
  float heading = atan2(hdgY, hdgX);
  if (heading < 0) {
	heading += radians(360);
  }
  return heading;
}

#endif