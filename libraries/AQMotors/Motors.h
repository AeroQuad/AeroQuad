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

  Motors()
  {
    _throttle = 0;
    _motorAxisCommand[ROLL] = 0;
    _motorAxisCommand[PITCH] = 0;
    _motorAxisCommand[YAW] = 0;
    for (byte motor = 0; motor < LASTMOTOR; motor++) 
    {
      _motorCommand[motor] = 1000;
      _minCommand[motor] = MINCOMMAND;
      _maxCommand[motor] = MAXCOMMAND;
      _remoteCommand[motor] = 1000;
    }
    _delta = 0;
  };

  // The following function calls must be defined in any new subclasses
  virtual void initialize();
  virtual void write ();
  virtual void commandAllMotors(int motorCommand);

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void pulseMotors(byte quantity) 
  {
    for (byte i = 0; i < quantity; i++) 
    {
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }

  void setRemoteCommand(byte motor, int value) 
  {
    _remoteCommand[motor] = value;
  }

  const int getRemoteCommand(byte motor) 
  {
    return _remoteCommand[motor];
  }

  const float getMotorSlope() 
  {
    return _mMotorCommand;
  }

  const float getMotorOffset() 
  {
    return _bMotorCommand;
  }

  void setMinCommand(byte motor, int value) 
  {
    _minCommand[motor] = value;
  }

  const int getMinCommand(byte motor) 
  {
    return _minCommand[motor];
  }

  void setMaxCommand(byte motor, int value) 
  {
    _maxCommand[motor] = value;
  }

  const int getMaxCommand(byte motor) 
  {
    return _maxCommand[motor];
  }

  void setMotorAxisCommand(byte motor, int value) 
  {
    _motorAxisCommand[motor] = value;
  }

  const int getMotorAxisCommand(byte motor) 
  {
    return _motorAxisCommand[motor];
  }

  void setMotorCommand(byte motor, int value) 
  {
    _motorCommand[motor] = value;
  }

  const int getMotorCommand(byte motor) 
  {
    return _motorCommand[motor];
  }
  
  void setThrottle(float value) 
  {
    _throttle = value;
  }
  
  const float getThrottle() 
  {
    return _throttle;
  }
};

#endif