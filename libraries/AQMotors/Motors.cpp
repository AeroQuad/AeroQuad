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

#include "Motors.h"

Motors::Motors()
{
  _throttle = 0;
  _motorAxisCommand[0] = 0;
  _motorAxisCommand[1] = 0;
  _motorAxisCommand[2] = 0;
  for (byte motor = 0; motor < LASTMOTOR; motor++) 
  {
    _motorCommand[motor] = 1000;
    _minCommand[motor] = MINCOMMAND;
    _maxCommand[motor] = MAXCOMMAND;
    _remoteCommand[motor] = 1000;
  }
};

// The following function calls must be defined in any new subclasses
void Motors::initialize() {}
void Motors::write() {}
void Motors::commandAllMotors(int motorCommand) {}

//Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
void Motors::pulseMotors(byte quantity) 
{
  for (byte i = 0; i < quantity; i++) 
  {
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}

void Motors::setRemoteCommand(byte motor, int value) 
{
  _remoteCommand[motor] = value;
}

const int Motors::getRemoteCommand(byte motor) 
{
  return _remoteCommand[motor];
}

const float Motors::getMotorSlope() 
{
  return _mMotorCommand;
}

const float Motors::getMotorOffset() 
{
  return _bMotorCommand;
}

void Motors::setMinCommand(byte motor, int value) 
{
  _minCommand[motor] = value;
}

const int Motors::getMinCommand(byte motor) 
{
  return _minCommand[motor];
}

void Motors::setMaxCommand(byte motor, int value) 
{
  _maxCommand[motor] = value;
}

const int Motors::getMaxCommand(byte motor) 
{
  return _maxCommand[motor];
}

void Motors::setMotorAxisCommand(byte motor, int value) 
{
  _motorAxisCommand[motor] = value;
}

const int Motors::getMotorAxisCommand(byte motor) 
{
  return _motorAxisCommand[motor];
}

void Motors::setMotorCommand(byte motor, int value) 
{
  _motorCommand[motor] = value;
}

const int Motors::getMotorCommand(byte motor) 
{
  return _motorCommand[motor];
}
  
void Motors::setThrottle(float value) 
{
  _throttle = value;
}
  
const float Motors::getThrottle() 
{
  return _throttle;
}
