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

#include "Receiver.h"

Receiver::Receiver() 
{
  _transmitterCommand[ROLL] = 1500;
  _transmitterCommand[PITCH] = 1500;
  _transmitterCommand[YAW] = 1500;
  _transmitterCommand[THROTTLE] = 1000;
  _transmitterCommand[MODE] = 1000;
  _transmitterCommand[AUX] = 1000;

  for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
  {
    _transmitterCommandSmooth[channel] = 1.0;
  }
  for (byte channel = ROLL; channel < THROTTLE; channel++)
  {
    _transmitterZero[channel] = 1500;
  }
}

// ******************************************************************
// The following function calls must be defined in any new subclasses
// ******************************************************************
void Receiver::initialize() {}
void Receiver::read() {}

// **************************************************************
// The following functions are common between all Gyro subclasses
// **************************************************************

void Receiver::_initialize() 
{
}

const int Receiver::getRaw(byte channel) 
{
  return _receiverData[channel];
}

const int Receiver::getData(byte channel) 
{
  // reduce sensitivity of transmitter input by xmitFactor
  return _transmitterCommand[channel];
}

const int Receiver::getTrimData(byte channel) 
{
  return _receiverData[channel] - _transmitterTrim[channel];
}

const int Receiver::getZero(byte channel) 
{
  return _transmitterZero[channel];
}

void Receiver::setZero(byte channel, int value) 
{
  _transmitterZero[channel] = value;
}

const int Receiver::getTransmitterTrim(byte channel) 
{
  return _transmitterTrim[channel];
}

void Receiver::setTransmitterTrim(byte channel, int value) 
{
  _transmitterTrim[channel] = value;
}

const float Receiver::getSmoothFactor(byte channel) 
{
  return _transmitterSmooth[channel];
}

void Receiver::setSmoothFactor(byte channel, float value) 
{
  _transmitterSmooth[channel] = value;
}

const float Receiver::getXmitFactor() 
{
  return _xmitFactor;
}

void Receiver::setXmitFactor(float value) 
{
  _xmitFactor = value;
}

const float Receiver::getTransmitterSlope(byte channel) 
{
  return _mTransmitter[channel];
}

void Receiver::setTransmitterSlope(byte channel, float value) 
{
  _mTransmitter[channel] = value;
}

const float Receiver::getTransmitterOffset(byte channel) 
{
  return _bTransmitter[channel];
}

void Receiver::setTransmitterOffset(byte channel, float value) 
{
  _bTransmitter[channel] = value;
}

const float Receiver::getAngle(byte channel) 
{
  // Scale 1000-2000 usecs to -45 to 45 degrees
  // m = 0.09, b = -135
  // reduce transmitterCommand by xmitFactor to lower sensitivity of transmitter input
  return (0.09 * _transmitterCommand[channel]) - 135;
}
