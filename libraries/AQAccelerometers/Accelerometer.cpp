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


#include "Accelerometer.h"

#include "AQMath.h"

// ******************************************************************
// Accelerometer
// ******************************************************************
Accelerometer::Accelerometer() 
{
  _sign[ROLL] = 1;
  _sign[PITCH] = 1;
  _sign[YAW] = 1;
}

// ******************************************************************
// The following function calls must be defined in any new subclasses
// ******************************************************************
void Accelerometer::initialize() 
{
  this->_initialize(_rollChannel, _pitchChannel, _zAxisChannel);
}
void Accelerometer::measure() {}
void Accelerometer::calibrate() {}
const int Accelerometer::getFlightData(byte) {}

// **************************************************************
// The following functions are common between all Gyro subclasses
// **************************************************************
void Accelerometer::_initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) 
{
  _accelChannel[ROLL] = rollChannel;
  _accelChannel[PITCH] = pitchChannel;
  _accelChannel[ZAXIS] = zAxisChannel;
  _currentAccelTime = micros();
  _previousAccelTime = _currentAccelTime;
}
  
const int Accelerometer::getRaw(byte axis) 
{
  return _accelADC[axis] * _sign[axis];
}
  
const int Accelerometer::getData(byte axis) 
{
  return _accelData[axis] * _sign[axis];
}
  
const int Accelerometer::invert(byte axis) 
{
  _sign[axis] = -_sign[axis];
  return _sign[axis];
}
  
const int Accelerometer::getZero(byte axis) 
{
  return _accelZero[axis];
}
  
void Accelerometer::setZero(byte axis, int value) 
{
  _accelZero[axis] = value;
}
  
const float Accelerometer::getScaleFactor() 
{
  return _accelScaleFactor;
}
  
const float Accelerometer::getSmoothFactor() 
{
  return _smoothFactor;
}
  
void Accelerometer::setSmoothFactor(float value) 
{
  _smoothFactor = value;
}
  
const float Accelerometer::angleRad(byte axis) 
{
  if (axis == PITCH) 
  {
    return arctan2(_accelData[PITCH] * _sign[PITCH], sqrt((long(_accelData[ROLL]) * _accelData[ROLL]) + (long(_accelData[ZAXIS]) * _accelData[ZAXIS])));
  }
  // then it have to be the ROLL axis
  return arctan2(_accelData[ROLL] * _sign[ROLL], sqrt((long(_accelData[PITCH]) * _accelData[PITCH]) + (long(_accelData[ZAXIS]) * _accelData[ZAXIS])));
}

const float Accelerometer::angleDeg(byte axis) 
{
  return degrees(angleRad(axis));
}
  
void Accelerometer::setOneG(int value) 
{
  _accelOneG = value;
}
  
const int Accelerometer::getOneG() 
{
  return _accelOneG;
}
  
const int Accelerometer::getZaxis() 
{
  //currentAccelTime = micros();
  //zAxis = filterSmoothWithTime(getFlightData(ZAXIS), zAxis, 0.25, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  //previousAccelTime = currentAccelTime;
  //return zAxis;
  return _accelOneG - getData(ZAXIS);
}
  
const float Accelerometer::getAltitude() 
{
  return _rawAltitude;
}
  
const float Accelerometer::rateG(const byte axis) 
{
  return getData(axis) / _accelOneG;
}
  
void Accelerometer::calculateAltitude(unsigned long currentTime) 
{
  _currentAccelTime = currentTime;
  if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
  {
    _rawAltitude += (getZaxis()) * ((_currentAccelTime - _previousAccelTime) / 1000000.0);
  }
  _previousAccelTime = currentTime;
} 
