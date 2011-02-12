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


class Gyroscope 
{
private:
  byte _rollChannel; 
  byte _pitchChannel;
  byte _yawChannel;
  int _sign[3];
  float _gyroHeading;

protected:
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    float _gyroZero[3];
  #else
    int _gyroZero[3];
  #endif
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
  Gyroscope()
  {
    _sign[ROLL] = 1;
    _sign[PITCH] = 1;
    _sign[YAW] = -1;
  }
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
  {
    this->_initialize(rollChannel, pitchChannel, yawChannel);
  }
  virtual void measure();
  virtual void calibrate();
  virtual void autoZero(){};
  virtual const int getFlightData(byte);
  virtual void initialize();

  // The following functions are common between all Gyro subclasses
  void _initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
  {
    _gyroChannel[ROLL] = rollChannel;
    _gyroChannel[PITCH] = pitchChannel;
    _gyroChannel[ZAXIS] = yawChannel;
    _previousTime = micros();
  }
    
  const int getRaw(byte axis) 
  {
    return _gyroADC[axis] * _sign[axis];
  }
  
  const int getData(byte axis) 
  {
    return _gyroData[axis] * _sign[axis];
  }
  
  void setData(byte axis, int value) 
  {
    _gyroData[axis] = value;
  }
  
  const int invert(byte axis) 
  {
    _sign[axis] = -_sign[axis];
    return _sign[axis];
  }
  
  const int getZero(byte axis) 
  {
    return _gyroZero[axis];
  }
  
  void setZero(byte axis, int value) 
  {
    _gyroZero[axis] = value;
  }    
  
  const float getScaleFactor() 
  {
    return _gyroScaleFactor;
  }

  const float getSmoothFactor() 
  {
    return _smoothFactor;
  }
  
  void setSmoothFactor(float value) 
  {
    _smoothFactor = value;
  }

  const float rateDegPerSec(byte axis) 
  {
    return ((_gyroADC[axis] * _sign[axis])) * _gyroScaleFactor;
  }

  const float rateRadPerSec(byte axis) 
  {
    return radians(rateDegPerSec(axis));
  }
  
  // returns heading as +/- 180 degrees
  const float getHeading() 
  {
    div_t integerDivide;
    
    integerDivide = div(_rawHeading, 360);
    _gyroHeading = _rawHeading + (integerDivide.quot * -360);
    if (_gyroHeading > 180)
    {
      _gyroHeading -= 360;
    }
    if (_gyroHeading < -180)
    {
      _gyroHeading += 360;
    }
    return _gyroHeading;
  }
  
  const float getRawHeading() 
  {
    return _rawHeading;
  }
  
  void setStartHeading(float value) 
  {
    // since a relative heading, get starting absolute heading from compass class
    _rawHeading = value;
  }
  
  void setReceiverYaw(int value) 
  {
    _receiverYaw = value;
  }
};

#endif