/*
  AeroQuad v3.0 - April 2011
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


#include "Compass.h"

#include <Axis.h>

Compass::Compass() {
}


void Compass::setMagCal(byte axis, float maxValue, float minValue) {
  magMax[axis] = maxValue;
  magMin[axis] = minValue;
  // Assume max/min is scaled to +1 and -1
  // y2 = 1, x2 = max; y1 = -1, x1 = min
  // m = (y2 - y1) / (x2 - x1)
  // m = 2 / (max - min)
  magScale[axis] = 2.0 / (magMax[axis] - magMin[axis]);
  // b = y1 - mx1; b = -1 - (m * min)
  magOffset[axis] = -(magScale[axis] * magMin[axis]) - 1;
}

const float Compass::getMagMax(byte axis) {
  return magMax[axis];
}

const float Compass::getMagMin(byte axis) {
  return magMin[axis];
}

const float Compass::getHdgXY(byte axis) {
  if (axis == XAXIS) return hdgX;
  if (axis == YAXIS) return hdgY;
}

const int Compass::getRawData(byte axis) {
  if (axis == XAXIS) return measuredMagX;
  if (axis == YAXIS) return measuredMagY;
  if (axis == ZAXIS) return measuredMagZ;
}





