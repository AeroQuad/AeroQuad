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

#ifndef _AQ_HMC5843_MAGNETOMETER_H_
#define _AQ_HMC5843_MAGNETOMETER_H_

#include "Compass.h"
#include <Gyroscope.h>

// ***********************************************************************
// ************************ HMC5843 Subclass *****************************
// ***********************************************************************
class HMC5843Magnetometer : public Compass 
{
// This sets up the HMC5843 from Sparkfun
private:
  Gyroscope *_gyro;
  float _cosRoll;
  float _sinRoll;
  float _cosPitch;
  float _sinPitch;
  float _magX;
  float _magY;
  int _measuredMagX;
  int _measuredMagY;
  int _measuredMagZ;
  float _smoothFactor; // time constant for complementary filter
  float _filter1;
  float _filter2; // coefficients for complementary filter
  float _adjustedGyroHeading;
  float _previousHead;
  int _gyroZero;
  
public: 
  HMC5843Magnetometer(Gyroscope *gyro);

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize();
  const int getRawData(byte axis);
  void measure(const float rollAngle, const float pitchAngle);
};

#endif