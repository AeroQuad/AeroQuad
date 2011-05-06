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

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// processHardManueversPlusMode ////////////////
//////////////////////////////////////////////////////////////////////////////
void processHardManueversPlusMode() {
  if (receiver->getData(ROLL) < MINCHECK) {
    motors->setMinCommand(LEFT, minAcro);
    motors->setMaxCommand(RIGHT, MAXCOMMAND);
  }
  else if (receiver->getData(ROLL) > MAXCHECK) {
    motors->setMaxCommand(LEFT, MAXCOMMAND);
    motors->setMinCommand(RIGHT, minAcro);
  }
  else if (receiver->getData(PITCH) < MINCHECK) {
    motors->setMaxCommand(FRONT, MAXCOMMAND);
    motors->setMinCommand(REAR, minAcro);
  }
  else if (receiver->getData(PITCH) > MAXCHECK) {
    motors->setMinCommand(FRONT, minAcro);
    motors->setMaxCommand(REAR, MAXCOMMAND);
  }
}

//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlPlusMode(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors->setMotorCommand(FRONT, throttle - motorAxisCommandPitch - motorAxisCommandYaw);
    motors->setMotorCommand(REAR, throttle + motorAxisCommandPitch - motorAxisCommandYaw);
    motors->setMotorCommand(RIGHT, throttle - motorAxisCommandRoll + motorAxisCommandYaw);
    motors->setMotorCommand(LEFT, throttle + motorAxisCommandRoll + motorAxisCommandYaw);
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    processHardManueversPlusMode();
  }

  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
    motors->setMotorCommand(motor, constrain(motors->getMotorCommand(motor), motors->getMinCommand(motor), motors->getMaxCommand(motor)));
  }

  // If throttle in minimum position, don't apply yaw
  if (receiver->getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
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

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
