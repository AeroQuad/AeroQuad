/*
  AeroQuad v3.0 - March 2011
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

#ifndef _AQ_ACCELEROMETER_H_
#define _AQ_ACCELEROMETER_H_

#include <WProgram.h>

class Accel {
public:
  #define XAXIS 0
  #define YAXIS 1
  #define ZAXIS 2
  #define LASTAXIS 3
  #define G_2_MPS2(g) (g * 9.80665)
  #define MPS2_2_G(m) (m * 0.10197162)
  #define FINDZERO 49
  
  float accelOneG;
  float accelScaleFactor;
  float accelVector[3];
  float smoothFactor;
  int   accelZero[3];
  int   accelRaw[3];

  Accel();
  virtual void initialize(void) {};
  virtual void measure(void) {};
  virtual void calibrate(void) {};
  
  const int getData(byte);
  void setZero(byte, int);
  const int getZero(byte);
  void setOneG(float);
  const float getOneG(void);
  void setSmoothFactor(float);
  int findMedian(int *, int);
};

#endif