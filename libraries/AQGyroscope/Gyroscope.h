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

#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_

#include <AQMath.h>

class Gyroscope {
public:
  float gyroFullScaleOutput;
  float gyroScaleFactor;
  float smoothFactor;
  int gyroChannel[3];
  int gyroData[3];
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float gyroZero[3];
  #else
    int gyroZero[3];
  #endif
  int gyroADC[3];
  byte rollChannel, pitchChannel, yawChannel;
  int sign[3];
  float rawHeading, gyroHeading;
  unsigned long currentTime, previousTime;
  
  // ************ Correct for gyro drift by FabQuad **************  
  // ************ http://aeroquad.com/entry.php?4-  **************     
  int lastReceiverYaw, receiverYaw;
  long yawAge;
  int positiveGyroYawCount;
  int negativeGyroYawCount;
  int zeroGyroYawCount;
    
  Gyroscope(void){
    sign[ROLL] = 1;
    sign[PITCH] = 1;
    sign[YAW] = -1;
  }
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual void measure(void);
  virtual void calibrate(void);
  virtual void autoZero(void){};
  virtual const int getFlightData(byte);

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel) {
    gyroChannel[ROLL] = rollChannel;
    gyroChannel[PITCH] = pitchChannel;
    gyroChannel[ZAXIS] = yawChannel;
    
    gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
    gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
    gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    
    previousTime = micros();
  }
    
  const int getRaw(byte axis) {
    return gyroADC[axis] * sign[axis];
  }
  
  const int getData(byte axis) {
    return gyroData[axis] * sign[axis];
  }
  
  void setData(byte axis, int value) {
    gyroData[axis] = value;
  }
  
  const int invert(byte axis) {
    sign[axis] = -sign[axis];
    return sign[axis];
  }
  
  const int getZero(byte axis) {
    return gyroZero[axis];
  }
  
  void setZero(byte axis, int value) {
    gyroZero[axis] = value;
  }    
  
  const float getScaleFactor() {
    return gyroScaleFactor;
  }

  const float getSmoothFactor(void) {
    return smoothFactor;
  }
  
  void setSmoothFactor(float value) {
    smoothFactor = value;
  }

  const float rateDegPerSec(byte axis) {
    return ((gyroADC[axis] * sign[axis])) * gyroScaleFactor;
  }

  const float rateRadPerSec(byte axis) {
    return radians(rateDegPerSec(axis));
  }
  
  // returns heading as +/- 180 degrees
  const float getHeading(void) {
    div_t integerDivide;
    
    integerDivide = div(rawHeading, 360);
    gyroHeading = rawHeading + (integerDivide.quot * -360);
    if (gyroHeading > 180) gyroHeading -= 360;
    if (gyroHeading < -180) gyroHeading += 360;
    return gyroHeading;
  }
  
  const float getRawHeading(void) {
    return rawHeading;
  }
  
  void setStartHeading(float value) {
    // since a relative heading, get starting absolute heading from compass class
    rawHeading = value;
  }
  
  void setReceiverYaw(int value) {
    receiverYaw = value;
  }
};

#endif