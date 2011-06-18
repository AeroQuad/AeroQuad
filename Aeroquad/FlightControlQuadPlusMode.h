/*
  AeroQuad v3.0 - April 2011
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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_

#define FRONT MOTOR1
#define REAR  MOTOR2
#define RIGHT MOTOR3
#define LEFT  MOTOR4
#define LASTMOTOR MOTOR4+1


void applyMotorCommand() {
  motors->setMotorCommand(FRONT, throttle - motorAxisCommandPitch - motorAxisCommandYaw);
  motors->setMotorCommand(REAR,  throttle + motorAxisCommandPitch - motorAxisCommandYaw);
  motors->setMotorCommand(RIGHT, throttle - motorAxisCommandRoll  + motorAxisCommandYaw);
  motors->setMotorCommand(LEFT,  throttle + motorAxisCommandRoll  + motorAxisCommandYaw);
}

void processMinMaxCommand() {
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorMaxCommand[motor] = MAXCOMMAND;
    motorMinCommand[motor] = MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
    motorMinCommand[RIGHT] = MAXCOMMAND;
    motorMaxCommand[LEFT] = minAcro;
  }
  else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
    motorMinCommand[LEFT] = MAXCOMMAND;
    motorMaxCommand[RIGHT] = minAcro;
  }
  else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
    motorMinCommand[FRONT] = MAXCOMMAND;
    motorMaxCommand[REAR] = minAcro;
  }
  else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
    motorMinCommand[REAR] = MAXCOMMAND;
    motorMaxCommand[FRONT] = minAcro;
  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
