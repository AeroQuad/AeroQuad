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
  
  Accelerometer(void) {
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = 1;
    zAxis = 0;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void) {
    this->_initialize(rollChannel, pitchChannel, zAxisChannel);
    smoothFactor = readFloat(ACCSMOOTH_ADR);
  }
  
  virtual void measure(void);
  virtual void calibrate(void);
  virtual const int getFlightData(byte);
  virtual void calculateAltitude(void);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************
  void _initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) {
    accelChannel[ROLL] = rollChannel;
    accelChannel[PITCH] = pitchChannel;
    accelChannel[ZAXIS] = zAxisChannel;
    accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
    accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
    accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
    accelOneG = readFloat(ACCEL1G_ADR);
    currentAccelTime = micros();
    previousAccelTime = currentAccelTime;
  }
  
  const int getRaw(byte axis) {
    return accelADC[axis] * sign[axis];
  }
  
  const int getData(byte axis) {
    return accelData[axis] * sign[axis];
  }
  
  const int invert(byte axis) {
    sign[axis] = -sign[axis];
    return sign[axis];
  }
  
  const int getZero(byte axis) {
    return accelZero[axis];
  }
  
  void setZero(byte axis, int value) {
    accelZero[axis] = value;
  }
  
  const float getScaleFactor(void) {
    return accelScaleFactor;
  }
  
  const float getSmoothFactor() {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }
  
  const float angleRad(byte axis) {
    if (axis == PITCH) return arctan2(accelData[PITCH] * sign[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
    if (axis == ROLL) return arctan2(accelData[ROLL] * sign[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }

  const float angleDeg(byte axis) {
    return degrees(angleRad(axis));
  }
  
  void setOneG(int value) {
    accelOneG = value;
  }
  
  const int getOneG(void) {
    return accelOneG;
  }
  
  const int getZaxis() {
    currentAccelTime = micros();
    zAxis = filterSmoothWithTime(getFlightData(ZAXIS), zAxis, 0.25, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
    previousAccelTime = currentAccelTime;
    return zAxis;
  }
  
  const float getAltitude(void) {
    return rawAltitude;
  }
  
  const float rateG(const byte axis) {
    return getData(axis) / accelOneG;
  }
};

#endif