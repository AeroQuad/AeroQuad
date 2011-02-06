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

#include <AxisDefine.h>
#include <EEPROMAddress.h>
#include <AQDataStorage.h>

#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define FINDZERO 9
#else
  #define FINDZERO 49
#endif


class Gyroscope 
{
public:
  float gyroFullScaleOutput;
  float gyroScaleFactor;
  float smoothFactor;
  int gyroChannel[3];
  int gyroData[3];
  float gyroZero[3];
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
    
  Gyroscope(void);
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel);
  virtual void measure(void);
  virtual void calibrate(void);
  virtual void autoZero(void);
  virtual const int getFlightData(byte);

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel);
    
  const int getRaw(byte axis);
  
  const int getData(byte axis);
  
  void setData(byte axis, int value);
  
  const int invert(byte axis);
  
  const int getZero(byte axis);
  
  void setZero(byte axis, int value);
  
  const float getScaleFactor();

  const float getSmoothFactor(void);
  
  void setSmoothFactor(float value);

  const float rateDegPerSec(byte axis);

  const float rateRadPerSec(byte axis);
  
  // returns heading as +/- 180 degrees
  const float getHeading(void);
  
  const float getRawHeading(void);
  
  void setStartHeading(float value);
  
  void setReceiverYaw(int value);
};

#endif