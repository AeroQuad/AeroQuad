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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_X_MODE_H_

#define FRONT_LEFT  MOTOR1
#define REAR_RIGHT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR_LEFT   MOTOR4
#define RIGHT       MOTOR5
#define LEFT        MOTOR6
#define LASTMOTOR   MOTOR6+1


void applyMotorCommand() {
  motors->setMotorCommand(FRONT_LEFT,  throttle - motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(REAR_RIGHT,  throttle + motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(FRONT_RIGHT, throttle - motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
  motors->setMotorCommand(REAR_LEFT,   throttle + motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(RIGHT,       throttle                         - motorAxisCommandRoll - motorAxisCommandYaw);
  motors->setMotorCommand(LEFT,        throttle                         + motorAxisCommandRoll + motorAxisCommandYaw);
}

void processMinMaxCommand() {
  if ((motors->getMotorCommand(FRONT_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(RIGHT) <= MINTHROTTLE)) {
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[REAR_RIGHT] =  constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[LEFT] =        constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motors->getMotorCommand(FRONT_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[REAR_RIGHT] =  constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[LEFT] =        constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[REAR_RIGHT] =  MAXCOMMAND;
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND; 
    motorMaxCommand[LEFT] =        MAXCOMMAND; 
    motorMinCommand[REAR_RIGHT] =  MINTHROTTLE;
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[LEFT] =        MINTHROTTLE;
  }

  if ((motors->getMotorCommand(REAR_RIGHT) <= MINTHROTTLE) || (motors->getMotorCommand(FRONT_RIGHT) <= MINTHROTTLE) || (motors->getMotorCommand(LEFT))){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =  constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[RIGHT] =      constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motors->getMotorCommand(REAR_RIGHT) >= MAXCOMMAND) || (motors->getMotorCommand(FRONT_RIGHT) >= MAXCOMMAND) || (motors->getMotorCommand(LEFT))) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT] =  constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[RIGHT] =      constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT] =  MAXCOMMAND;
    motorMaxCommand[RIGHT] =      MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT] =  MINTHROTTLE;
    motorMinCommand[RIGHT] =      MINTHROTTLE;
  }
}

void processHardManuevers() {
  if (flightMode == ACRO) {
    if (receiver->getData(ROLL) < MINCHECK) {        // Maximum Left Roll Rate
      motorMinCommand[RIGHT] =       MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMinCommand[REAR_RIGHT] =  MAXCOMMAND;
      motorMaxCommand[LEFT] =        minAcro;
      motorMaxCommand[FRONT_LEFT] =  minAcro;
      motorMaxCommand[REAR_LEFT] =   minAcro;
    }
    else if (receiver->getData(ROLL) > MAXCHECK) {   // Maximum Right Roll Rate
      motorMinCommand[LEFT] =        MAXCOMMAND;
      motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
      motorMinCommand[REAR_LEFT] =   MAXCOMMAND;
      motorMaxCommand[RIGHT] =       minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
      motorMaxCommand[REAR_RIGHT] =  minAcro;
    }
    else if (receiver->getData(PITCH) < MINCHECK) {  // Maximum Nose Up Pitch Rate
      motorMinCommand[FRONT_LEFT] =  MAXCOMMAND;
      motorMinCommand[FRONT_RIGHT] = MAXCOMMAND;
      motorMaxCommand[REAR_LEFT] =   minAcro;
      motorMaxCommand[REAR_RIGHT] =  minAcro;
    }
    else if (receiver->getData(PITCH) > MAXCHECK) {  // Maximum Nose Down Pitch Rate
      motorMinCommand[REAR_LEFT] =   MAXCOMMAND;
      motorMinCommand[REAR_RIGHT] =  MAXCOMMAND;
      motorMaxCommand[FRONT_LEFT] =  minAcro;
      motorMaxCommand[FRONT_RIGHT] = minAcro;
    }
  }
}

#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_HEX_X_MODE_H_
