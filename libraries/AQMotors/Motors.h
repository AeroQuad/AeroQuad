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

#ifndef _AQ_MOTORS_H_
#define _AQ_MOTORS_H_

#include "WProgram.h"

#if defined(plusConfig) || defined(XConfig)
  #define LASTMOTOR 4
#endif
#if defined(HEXACOAXIAL) || defined(HEXARADIAL)
  #define LASTMOTOR 6
#else
  #define LASTMOTOR 4
#endif

#define ROLL 0
#define PITCH 1
#define YAW 2
#define MINCOMMAND 1000
#define MAXCOMMAND 2000


class Motors 
{
private:
  // Assume maximum number of motors is 8, leave array indexes unused if lower number
  int _motorAxisCommand[3];
  int _minCommand[LASTMOTOR];
  int _maxCommand[LASTMOTOR];
  float _throttle;
  // Ground station control
  int _remoteCommand[LASTMOTOR];

protected:
  int _motorCommand[LASTMOTOR];
  float _mMotorCommand;
  float _bMotorCommand;
  
public:

  Motors();

  // The following function calls must be defined in any new subclasses
  virtual void initialize();
  virtual void write ();
  virtual void commandAllMotors(int motorCommand);

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void pulseMotors(byte quantity);

  void setRemoteCommand(byte motor, int value);
  const int getRemoteCommand(byte motor);
  const float getMotorSlope();
  const float getMotorOffset();
  void setMinCommand(byte motor, int value);
  const int getMinCommand(byte motor);
  void setMaxCommand(byte motor, int value);
  const int getMaxCommand(byte motor);
  void setMotorAxisCommand(byte motor, int value);
  const int getMotorAxisCommand(byte motor);
  void setMotorCommand(byte motor, int value);
  const int getMotorCommand(byte motor);
  void setThrottle(float value);
  const float getThrottle();
};

#endif