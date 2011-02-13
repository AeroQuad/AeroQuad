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

#ifndef _AQ_ACCELEROMETER_H_
#define _AQ_ACCELEROMETER_H_

#include "WProgram.h"

#define ROLL 0
#define PITCH 1
#define YAW 2
#define ZAXIS 2
#define LASTAXIS 3

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define FINDZERO 9
#else
  #define FINDZERO 49
#endif


class Accelerometer 
{
private:
  int _sign[3];
  byte _rollChannel;
  byte _pitchChannel;
  byte _zAxisChannel;
  
protected:  
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float _accelZero[3];
  #else
    int _accelZero[3];
  #endif
  int _accelChannel[3];
  int _accelADC[3];
  int _accelData[3];  
  float _accelScaleFactor;
  float _accelOneG;
  float _smoothFactor;
  float _rawAltitude;  
  
  unsigned long _currentAccelTime;
  unsigned long _previousAccelTime;



  
public:  
  // ******************************************************************
  // Constructor
  // ******************************************************************
  Accelerometer();

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize();
  virtual void measure();
  virtual void calibrate();
  virtual const int getFlightData(byte);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel);
  const int getRaw(byte axis);
  const int getData(byte axis);
  const int invert(byte axis);
  const int getZero(byte axis);
  void setZero(byte axis, int value);
  const float getScaleFactor();
  const float getSmoothFactor();
  void setSmoothFactor(float value);
  const float angleRad(byte axis);
  const float angleDeg(byte axis);
  void setOneG(int value);
  const int getOneG();
  const int getZaxis();
  const float getAltitude();
  const float rateG(const byte axis);
  virtual void calculateAltitude();
};

#endif