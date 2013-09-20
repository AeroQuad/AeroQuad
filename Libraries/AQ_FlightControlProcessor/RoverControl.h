/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_PROCESS_ROVER_CONTROL_H_
#define _AQ_PROCESS_ROVER_CONTROL_H_

#include "FlightControlVariable.h"

#define SPEED       MOTOR1
#define STEERING    MOTOR2
#define TILTCAM     MOTOR3
#define PANCAM      MOTOR4
#define LASTMOTOR   (MOTOR4+1)

#define TRI_YAW_CONSTRAINT_MIN 1100
#define TRI_YAW_CONSTRAINT_MAX 1900
#define TRI_YAW_MIDDLE 1500

#define MAX_RECEIVER_OFFSET 50

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  motorCommand[SPEED] = motorAxisCommandPitch;
  motorCommand[STEERING] = motorAxisCommandRoll;
  motorCommand[TILTCAM]  = receiverCommand[THROTTLE];
  motorCommand[PANCAM]   = receiverCommand[ZAXIS];
}

#endif
