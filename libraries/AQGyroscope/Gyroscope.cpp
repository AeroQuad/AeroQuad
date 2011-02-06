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

#include <Gyroscope.h>
    
Gyroscope::Gyroscope(void)
{
  sign[ROLL] = 1;
  sign[PITCH] = 1;
  sign[YAW] = -1;
}
  
// The following function calls must be defined in any new subclasses
void Gyroscope::initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
{
  this->_initialize(rollChannel, pitchChannel, yawChannel);
}

void Gyroscope::measure(void){}
void Gyroscope::calibrate(void){}
void Gyroscope::autoZero(void){}
const int Gyroscope::getFlightData(byte){}

// The following functions are common between all Gyro subclasses
void Gyroscope::_initialize(byte rollChannel, byte pitchChannel, byte yawChannel) 
{
  gyroChannel[ROLL] = rollChannel;
  gyroChannel[PITCH] = pitchChannel;
  gyroChannel[ZAXIS] = yawChannel;
    
  gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
    
  previousTime = micros();
}
    
const int Gyroscope::getRaw(byte axis) 
{
  return gyroADC[axis] * sign[axis];
}
  
const int Gyroscope::getData(byte axis) 
{
  return gyroData[axis] * sign[axis];
}
  
void Gyroscope::setData(byte axis, int value) 
{
  gyroData[axis] = value;
}
  
const int Gyroscope::invert(byte axis) 
{
  sign[axis] = -sign[axis];
  return sign[axis];
}
  
const int Gyroscope::getZero(byte axis) 
{
  return gyroZero[axis];
}
  
void Gyroscope::setZero(byte axis, int value) 
{
  gyroZero[axis] = value;
}    
  
const float Gyroscope::getScaleFactor() 
{
  return gyroScaleFactor;
}

const float Gyroscope::getSmoothFactor(void) 
{
  return smoothFactor;
}
  
void Gyroscope::setSmoothFactor(float value) 
{
  smoothFactor = value;
}

const float Gyroscope::rateDegPerSec(byte axis) 
{
  return ((gyroADC[axis] * sign[axis])) * gyroScaleFactor;
}

const float Gyroscope::rateRadPerSec(byte axis) 
{
  return radians(rateDegPerSec(axis));
}
  
// returns heading as +/- 180 degrees
const float Gyroscope::getHeading(void) 
{
  div_t integerDivide;
    
  integerDivide = div(rawHeading, 360);
  gyroHeading = rawHeading + (integerDivide.quot * -360);
  if (gyroHeading > 180) gyroHeading -= 360;
  if (gyroHeading < -180) gyroHeading += 360;
  return gyroHeading;
}
  
const float Gyroscope::getRawHeading(void) 
{
  return rawHeading;
}
  
void Gyroscope::setStartHeading(float value) 
{
  // since a relative heading, get starting absolute heading from compass class
  rawHeading = value;
}
  
void Gyroscope::setReceiverYaw(int value) 
{
  receiverYaw = value;
}
