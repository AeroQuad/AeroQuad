/*
  AeroQuad v2.2 - Feburary 2011
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

#ifndef _AQ_COMPASS_H_
#define _AQ_COMPASS_H_

#include "WProgram.h"

// Class to define sensors that can determine absolute heading

// ***********************************************************************
// ************************** Compass Class ******************************
// ***********************************************************************
class Compass 
{
private:  
  float _magMax[3];
  float _magMin[3];
  
protected:  
  float _absoluteHeading; 
  float _heading; 
  float _compass;
  int _compassAddress;
  float _gyroStartHeading;
  float _magScale[3];
  float _magOffset[3];
  
public: 

  Compass();

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(); 
  virtual void measure(const float rollAngle, const float pitchAngle);
  virtual const int getRawData(byte);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData();
  const float getHeading();
  const float getAbsoluteHeading();
  void setMagCal(byte axis, float maxValue, float minValue);
  const float getMagMax(byte axis);
  const float getMagMin(byte axis);
};

#endif