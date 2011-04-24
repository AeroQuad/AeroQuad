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

#include "Accelerometer.h"

Accelerometer::Accelerometer() {
}

const float Accelerometer::getSmoothFactor() {
  return smoothFactor;
}

void Accelerometer::setSmoothFactor(float value) {
  smoothFactor = value;
}

void Accelerometer::setOneG(float oneG) {
  this->oneG = oneG;
}

float Accelerometer::getOneG() {
  return oneG;
}

float Accelerometer::getMeterPerSec(byte axis) {
  return meterPerSec[axis];
}

float Accelerometer::getZero(byte axis) {
  return zero[axis];
}

void Accelerometer::setZero(byte axis, float zero) {
  this->zero[axis] = zero;
}