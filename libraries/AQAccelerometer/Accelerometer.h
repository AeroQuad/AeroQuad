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

#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

#include <AQMath.h>

#include <AxisDefine.h>
#include <EEPROMAddress.h>
#include <AQDataStorage.h>


#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define FINDZERO 9
#else
  #define FINDZERO 49
#endif


class Accelerometer {
public:
  float accelScaleFactor;
  float smoothFactor;
  float rawAltitude;
  int accelChannel[3];
  float accelZero[3];
  int accelData[3];
  int accelADC[3];
  int sign[3];
  float accelOneG, zAxis;
  byte rollChannel, pitchChannel, zAxisChannel;
  unsigned long currentAccelTime, previousAccelTime;
  
  Accelerometer(void);

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void);
  virtual void measure(void);
  virtual void calibrate(void);
  virtual const int getFlightData(byte);
  virtual void calculateAltitude(void);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel);
  
  const int getRaw(byte axis);
  
  const int getData(byte axis);
  
  const int invert(byte axis);
  
  const int getZero(byte axis);
  
  void setZero(byte axis, int value);
  
  const float getScaleFactor(void);
  
  const float getSmoothFactor();
  
  void setSmoothFactor(float value);
  
  const float angleRad(byte axis);

  const float angleDeg(byte axis);
  
  void setOneG(int value);
  
  const int getOneG(void);
  
  const int getZaxis(unsigned long currentTime, unsigned long previousTime);
  
  const float getAltitude(void);
  
  const float rateG(const byte axis);
};

#endif