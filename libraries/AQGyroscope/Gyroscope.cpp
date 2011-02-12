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

#include "Gyroscope.h"

Gyroscope::Gyroscope()
{
  _sign[ROLL] = 1;
  _sign[PITCH] = 1;
  _sign[YAW] = -1;
}
  
// The following function calls must be defined in any new subclasses
void Gyroscope::initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
{
  this->_initialize(rollChannel, pitchChannel, yawChannel);
}
void Gyroscope::measure() {}
void Gyroscope::calibrate() {}
void Gyroscope::autoZero() {}
const int Gyroscope::getFlightData(byte) {}
void Gyroscope::initialize() {}

// The following functions are common between all Gyro subclasses
void Gyroscope::_initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
{
  _gyroChannel[ROLL] = rollChannel;
  _gyroChannel[PITCH] = pitchChannel;
  _gyroChannel[ZAXIS] = yawChannel;
  _previousGyroTime = micros();
}
    
const int Gyroscope::getRaw(byte axis) 
{
  return _gyroADC[axis] * _sign[axis];
}
  
const int Gyroscope::getData(byte axis) 
{
  return _gyroData[axis] * _sign[axis];
}
  
void Gyroscope::setData(byte axis, int value) 
{
  _gyroData[axis] = value;
}
  
const int Gyroscope::invert(byte axis) 
{
  _sign[axis] = -_sign[axis];
  return _sign[axis];
}
 
const int Gyroscope::getZero(byte axis) 
{
  return _gyroZero[axis];
}
  
void Gyroscope::setZero(byte axis, int value) 
{
  _gyroZero[axis] = value;
}    
  
const float Gyroscope::getScaleFactor() 
{
  return _gyroScaleFactor;
}

const float Gyroscope::getSmoothFactor() 
{
  return _smoothFactor;
}
  
void Gyroscope::setSmoothFactor(float value) 
{
  _smoothFactor = value;
}

const float Gyroscope::rateDegPerSec(byte axis) 
{
  return ((_gyroADC[axis] * _sign[axis])) * _gyroScaleFactor;
}

const float Gyroscope::rateRadPerSec(byte axis) 
{
  return radians(rateDegPerSec(axis));
}
  
// returns heading as +/- 180 degrees
const float Gyroscope::getHeading() 
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
  
const float Gyroscope::getRawHeading() 
{
  return _rawHeading;
}
  
void Gyroscope::setStartHeading(float value) 
{
  // since a relative heading, get starting absolute heading from compass class
  _rawHeading = value;
}
  
void Gyroscope::setReceiverYaw(int value) 
{
  _receiverYaw = value;
}
