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

#define FRONT_LEFT  0
#define REAR_RIGHT  1
#define FRONT_RIGHT 2
#define REAR_LEFT   3
#define LASTMOTOR   4


void processFlightControl(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
    motors->setMotorCommand(FRONT_LEFT,  throttle - motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
    motors->setMotorCommand(FRONT_RIGHT, throttle - motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
    motors->setMotorCommand(REAR_LEFT,   throttle + motorAxisCommandPitch + motorAxisCommandRoll + motorAxisCommandYaw);
    motors->setMotorCommand(REAR_RIGHT,  throttle + motorAxisCommandPitch - motorAxisCommandRoll - motorAxisCommandYaw);
  } 

  // *********************** process min max motor command *******************

  if ((motors->getMotorCommand(FRONT_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(REAR_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_LEFT] =   constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motors->getMotorCommand(FRONT_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(REAR_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_RIGHT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_LEFT]   = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_RIGHT] = MAXCOMMAND;
    motorMaxCommand[REAR_LEFT]   = MAXCOMMAND; 
    motorMinCommand[FRONT_RIGHT] = MINTHROTTLE;
    motorMinCommand[REAR_LEFT]   = MINTHROTTLE;
  }

  if ((motors->getMotorCommand(REAR_LEFT) <= MINTHROTTLE) || (motors->getMotorCommand(FRONT_RIGHT) <= MINTHROTTLE)){
    delta = receiver->getData(THROTTLE) - MINTHROTTLE;
    motorMaxCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
    motorMaxCommand[REAR_RIGHT] = constrain(receiver->getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motors->getMotorCommand(REAR_LEFT) >= MAXCOMMAND) || (motors->getMotorCommand(FRONT_RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver->getData(THROTTLE);
    motorMinCommand[FRONT_LEFT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
    motorMinCommand[REAR_RIGHT] = constrain(receiver->getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    motorMaxCommand[FRONT_LEFT] = MAXCOMMAND;
    motorMaxCommand[REAR_RIGHT] = MAXCOMMAND;
    motorMinCommand[FRONT_LEFT] = MINTHROTTLE;
    motorMinCommand[REAR_RIGHT] = MINTHROTTLE;
  }

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers

  if (flightMode == ACRO) {
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
  
  // Apply limits to motor commands
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motors->setMotorCommand(motor, constrain(motors->getMotorCommand(motor), motorMinCommand[motor], motorMaxCommand[motor]));
  }

  // If throttle in minimum position, don't apply yaw
  if (receiver->getData(THROTTLE) < MINCHECK) {
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motors->setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors->write(); // Defined in Motors.h
  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
