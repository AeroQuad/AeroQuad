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
  if ((motors->getMotorCommand(FRONT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[RIGHT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[LEFT] =  constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motors->getMotorCommand(FRONT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[RIGHT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[LEFT] =  constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[RIGHT] = MAXCOMMAND;
    motorMaxCommand[LEFT] =  MAXCOMMAND;
    motorMinCommand[RIGHT] = MINTHROTTLE;
    motorMinCommand[LEFT] =  MINTHROTTLE;
  }

  if ((motors->getMotorCommand(LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE; 
    motorMaxCommand[FRONT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR]  = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motors->getMotorCommand(LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR]  = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT] = MAXCOMMAND; 
    motorMaxCommand[REAR]  = MAXCOMMAND;
    motorMinCommand[FRONT] = MINTHROTTLE;
    motorMinCommand[REAR]  = MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
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
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
