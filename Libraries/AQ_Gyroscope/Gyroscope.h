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

#ifndef _AEROQUAD_GYROSCOPE_H_
#define _AEROQUAD_GYROSCOPE_H_

#include <WProgram.h>
#include <Axis.h>

class Gyroscope {
protected:
  float scaleFactor;
  float rate[3];
  int zero[3];
  float smoothFactor;
  float gyroScaleFactor;
  float heading;
  unsigned long lastMesuredTime;
  
public:  
  Gyroscope() {}

  virtual void initialize() {}
  virtual void measure() {}
  virtual void calibrate() {}



  void setZero(byte axis,float value) {
    zero[axis] = value;
  }

  const float getZero(byte axis) {
    return zero[axis];
  }
  
  const float getSmoothFactor() {
    return smoothFactor;
  }

  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

  const float getRadPerSec(byte axis) {
    return rate[axis];
  }

  const float getHeading() {
    return heading;
  }
};
#endif