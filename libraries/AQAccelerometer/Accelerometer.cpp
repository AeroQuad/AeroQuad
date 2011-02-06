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

#include <Accelerometer.h>
  
Accelerometer::Accelerometer(void) 
{
  sign[ROLL] = 1;
  sign[PITCH] = 1;
  sign[YAW] = 1;
  zAxis = 0;
}

// ******************************************************************
// The following function calls must be defined in any new subclasses
// ******************************************************************
void Accelerometer::initialize(void) 
{
  this->_initialize(rollChannel, pitchChannel, zAxisChannel);
  smoothFactor = readFloat(ACCSMOOTH_ADR);
}

void Accelerometer::measure(void) {}
void Accelerometer::calibrate(void) {}
const int Accelerometer::getFlightData(byte) {}
void Accelerometer::calculateAltitude(void) {}


// **************************************************************
// The following functions are common between all Gyro subclasses
// **************************************************************
void Accelerometer::_initialize(byte rollChannel, byte pitchChannel, byte zAxisChannel) 
{
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

const int Accelerometer::getRaw(byte axis) 
{
  return accelADC[axis] * sign[axis];
}

const int Accelerometer::getData(byte axis) 
{
  return accelData[axis] * sign[axis];
}

const int Accelerometer::invert(byte axis) 
{
  sign[axis] = -sign[axis];
  return sign[axis];
}

const int Accelerometer::getZero(byte axis) 
{
  return accelZero[axis];
}

void Accelerometer::setZero(byte axis, int value) 
{
  accelZero[axis] = value;
}

const float Accelerometer::getScaleFactor(void) 
{
  return accelScaleFactor;
}

const float Accelerometer::getSmoothFactor() 
{
  return smoothFactor;
}

void Accelerometer::setSmoothFactor(float value) 
{
  smoothFactor = value;
}

const float Accelerometer::angleRad(byte axis) 
{
  if (axis == PITCH) 
  {
    return arctan2(accelData[PITCH] * sign[PITCH], sqrt((long(accelData[ROLL]) * accelData[ROLL]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }
  if (axis == ROLL) 
  {
    return arctan2(accelData[ROLL] * sign[ROLL], sqrt((long(accelData[PITCH]) * accelData[PITCH]) + (long(accelData[ZAXIS]) * accelData[ZAXIS])));
  }
}

const float Accelerometer::angleDeg(byte axis) 
{
  return degrees(angleRad(axis));
}

void Accelerometer::setOneG(int value) 
{
  accelOneG = value;
}

const int Accelerometer::getOneG(void) 
{
  return accelOneG;
}

const int Accelerometer::getZaxis(unsigned long currentTime, unsigned long previousTime) 
{
  currentAccelTime = micros();
  zAxis = filterSmoothWithTime(getFlightData(ZAXIS), zAxis, 0.25, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Âµs = (current-previous) / 5000.0 to get around 1
  previousAccelTime = currentAccelTime;
  return zAxis;
}

const float Accelerometer::getAltitude(void) 
{
  return rawAltitude;
}

const float Accelerometer::rateG(const byte axis) 
{
  return getData(axis) / accelOneG;
}

