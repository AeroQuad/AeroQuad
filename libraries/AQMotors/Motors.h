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

#ifndef _MOTORS_H_
#define _MOTORS_H_

class Motors {
public:
  // Assume maximum number of motors is 8, leave array indexes unused if lower number
  int motorAxisCommand[3];
  //int motorAxisCommandRoll[LASTMOTOR];
  //int motorAxisCommandPitch[LASTMOTOR];
  //int motorAxisCommandYaw[LASTMOTOR];
  //int motorMixerSettingRoll[LASTMOTOR];
  //int motorMixerSettingPitch[LASTMOTOR];
  //int motorMixerSettingYaw[LASTMOTOR];
  int motorCommand[LASTMOTOR];
  int minCommand[LASTMOTOR];
  int maxCommand[LASTMOTOR];
  float throttle;
  float timerDebug;
  int delta;
  byte axis;
  // Ground station control
  int remoteCommand[LASTMOTOR];
  float mMotorCommand;
  float bMotorCommand;


  Motors(void){
    throttle = 0;
    motorAxisCommand[ROLL] = 0;
    motorAxisCommand[PITCH] = 0;
    motorAxisCommand[YAW] = 0;
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      //motorAxisCommandRoll[motor] = 0;
      //motorAxisCommandPitch[motor] = 0;
      //motorAxisCommandYaw[motor] = 0;
      //motorMixerSettingRoll[motor] = 0;
      //motorMixerSettingPitch[motor] = 0;
      //motorMixerSettingYaw[motor] = 0;
      motorCommand[motor] = 1000;
      minCommand[motor] = MINCOMMAND;
      maxCommand[motor] = MAXCOMMAND;
      remoteCommand[motor] = 1000;
    }
    delta = 0;
  };

  // The following function calls must be defined in any new subclasses
  virtual void initialize(void);
  virtual void write (void);
  virtual void commandAllMotors(int motorCommand);

  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void pulseMotors(byte quantity) {
    for (byte i = 0; i < quantity; i++) {
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }

  void setRemoteCommand(byte motor, int value) {
    remoteCommand[motor] = value;
  }

  const int getRemoteCommand(byte motor) {
    return remoteCommand[motor];
  }

  const float getMotorSlope(void) {
    return mMotorCommand;
  }

  const float getMotorOffset(void) {
    return bMotorCommand;
  }

  void setMinCommand(byte motor, int value) {
    minCommand[motor] = value;
  }

  const int getMinCommand(byte motor) {
    return minCommand[motor];
  }

  void setMaxCommand(byte motor, int value) {
    maxCommand[motor] = value;
  }

  const int getMaxCommand(byte motor) {
    return maxCommand[motor];
  }

  void setMotorAxisCommand(byte motor, int value) {
    motorAxisCommand[motor] = value;
  }

  const int getMotorAxisCommand(byte motor) {
    return motorAxisCommand[motor];
  }

  void setMotorCommand(byte motor, int value) {
    motorCommand[motor] = value;
  }

  const int getMotorCommand(byte motor) {
    return motorCommand[motor];
  }
  void setThrottle(float value) {
    throttle = value;
  }
  const float getThrottle() {
    return throttle;
  }
};

#endif