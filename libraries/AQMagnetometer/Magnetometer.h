/*
  AeroQuad v2.1 - January 2011
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

// Class to define sensors that can determine absolute heading

#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

#include "WProgram.h"

// ***********************************************************************
// ************************** Magnetometer Class *************************
// ***********************************************************************
class Magnetometer {
public: 
  int compassAddress;
  float heading, absoluteHeading, gyroStartHeading;
  float compass;
  float magMax[3];
  float magMin[3];
  float magScale[3];
  float magOffset[3];

  Magnetometer(void);
  
  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void measure(float angleRoll, float anglePitch); 
  virtual const int getRawData(byte axis);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData(void);
  
  const float getHeading(void);
  
  const float getAbsoluteHeading(void);
  
  void setMagCal(byte axis, float maxValue, float minValue);
  
  const float getMagMax(byte axis);
  
  const float getMagMin(byte axis);
};

#endif