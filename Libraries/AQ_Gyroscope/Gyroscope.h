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

#ifndef _AQ_GYROSCOPE_H_
#define _AQ_GYROSCOPE_H_

#include "WProgram.h"

#include <Axis.h>

class Gyroscope 
{
private:
  byte _rollChannel; 
  byte _pitchChannel;
  byte _yawChannel;
  int _sign[3];
  float _gyroHeading;

protected:
  float _gyroZero[3];
  int _gyroADC[3];
  int _gyroChannel[3];  
  int _gyroData[3];  
  int _lastReceiverYaw;  
  int _positiveGyroYawCount; 
  int _negativeGyroYawCount;  
  int _zeroGyroYawCount;  
  int _receiverYaw;  
  // ************ Correct for gyro drift by FabQuad **************  
  // ************ http://aeroquad.com/entry.php?4-  **************     
  long _yawAge;
  float _gyroFullScaleOutput;
  float _gyroScaleFactor;
  float _smoothFactor;  
  float _rawHeading;  

  unsigned long _currentGyroTime;
  unsigned long _previousGyroTime;
  
public:    
  Gyroscope();
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel);
  virtual void measure();
  virtual void calibrate();
  virtual void autoZero();
  virtual void initialize();
  virtual const int getFlightData(byte axis);

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel);
  const int getRaw(byte axis);
  const int getData(byte axis);
  void setData(byte axis, int value);
  const int invert(byte axis);
  const int getZero(byte axis);
  void setZero(byte axis, int value);
  const float getScaleFactor();
  const float getSmoothFactor();
  void setSmoothFactor(float value);
  const float rateDegPerSec(byte axis);
  const float rateRadPerSec(byte axis);
  // returns heading as +/- 180 degrees
  const float getHeading();
  const float getRawHeading();
  void setStartHeading(float value);
  void setReceiverYaw(int value);
};

#endif