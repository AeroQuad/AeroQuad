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

#ifndef _MAGNETOMETER_HMC5843_H_
#define _MAGNETOMETER_HMC5843_H_

#include <Magnetometer.h>
#include <Gyroscope.h>

// ***********************************************************************
// ************************ HMC5843 Subclass *****************************
// ***********************************************************************
class MagnetometerHMC5843 : public Magnetometer {
// This sets up the HMC5843 from Sparkfun
private:
  float cosRoll;
  float sinRoll;
  float cosPitch;
  float sinPitch;
  float magX;
  float magY;
  int measuredMagX;
  int measuredMagY;
  int measuredMagZ;
  float smoothFactor; // time constant for complementary filter
  float filter1, filter2; // coefficients for complementary filter
  float adjustedGyroHeading, previousHead;
  int gyroZero;
  Gyroscope *_gyroscope;
  
public: 

  MagnetometerHMC5843(Gyroscope gyroscope);

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void);
  
  const int getRawData(byte axis);
  
  void measure(float angleRoll, float anglePitch);
};

#endif