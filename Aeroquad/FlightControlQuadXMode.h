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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

#define FRONT_LEFT  MOTOR1
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_LEFT   MOTOR4
#define LASTMOTOR   MOTOR4+1

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  motors->setMotorCommand(FRONT_LEFT,  throttle - motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(FRONT_RIGHT, throttle - motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(REAR_LEFT,   throttle + motorAxisCommandPitch + motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(REAR_RIGHT,  throttle + motorAxisCommandPitch - motorAxisCommandRoll - motorAxisCommandYaw);
}

void processMinMaxCommand() {
  
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorMaxCommand[motor] = MAXCOMMAND;
    motorMinCommand[motor] = MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
    motorMinCommand[FRONT_RIGHT] =MAXCOMMAND;
    motorMinCommand[REAR_RIGHT] = MAXCOMMAND;
    motorMaxCommand[FRONT_LEFT] = minAcro;
    motorMaxCommand[REAR_LEFT]  = minAcro;
  }
  else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
    motorMinCommand[FRONT_LEFT]  = MAXCOMMAND;
    motorMinCommand[REAR_LEFT]   = MAXCOMMAND;
    motorMaxCommand[FRONT_RIGHT] = minAcro;
    motorMaxCommand[REAR_RIGHT]  = minAcro;
  }
  else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
    motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
    motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]   = minAcro;
    motorMaxCommand[REAR_RIGHT]  = minAcro;
  }
  else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
    motorMinCommand[REAR_LEFT]   = MAXCOMMAND;
    motorMinCommand[REAR_RIGHT]  = MAXCOMMAND;
    motorMaxCommand[FRONT_LEFT]  = minAcro;
    motorMaxCommand[FRONT_RIGHT] = minAcro;
  }
}



#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
