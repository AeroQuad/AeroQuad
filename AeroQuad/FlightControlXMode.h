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

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// processHardManueversXMode ///////////////////
//////////////////////////////////////////////////////////////////////////////
void processHardManueversXMode()
{
  if (receiver->getData(ROLL) < MINCHECK) {
    motors->setMaxCommand(FRONT, minAcro);
    motors->setMaxCommand(REAR, MAXCOMMAND);
    motors->setMaxCommand(LEFT, minAcro);
    motors->setMaxCommand(RIGHT, MAXCOMMAND);
  }
  else if (receiver->getData(ROLL) > MAXCHECK) {
    motors->setMaxCommand(FRONT, MAXCOMMAND);
    motors->setMaxCommand(REAR, minAcro);
    motors->setMaxCommand(LEFT, MAXCOMMAND);
    motors->setMaxCommand(RIGHT, minAcro);
  }
  else if (receiver->getData(PITCH) < MINCHECK) {
    motors->setMaxCommand(FRONT, MAXCOMMAND);
    motors->setMaxCommand(REAR, minAcro);
    motors->setMaxCommand(LEFT, minAcro);
    motors->setMaxCommand(RIGHT, MAXCOMMAND);
  }
  else if (receiver->getData(PITCH) > MAXCHECK) {
    motors->setMaxCommand(FRONT, minAcro);
    motors->setMaxCommand(REAR, MAXCOMMAND);
    motors->setMaxCommand(LEFT, MAXCOMMAND);
    motors->setMaxCommand(RIGHT, minAcro);
  }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// X MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlXMode(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
    motors->setMotorCommand(FRONT, throttle - motorAxisCommandPitch + motorAxisCommandRoll - motorAxisCommandYaw);
    motors->setMotorCommand(RIGHT, throttle - motorAxisCommandPitch - motorAxisCommandRoll + motorAxisCommandYaw);
    motors->setMotorCommand(LEFT, throttle + motorAxisCommandPitch + motorAxisCommandRoll + motorAxisCommandYaw);
    motors->setMotorCommand(REAR, throttle + motorAxisCommandPitch - motorAxisCommandRoll - motorAxisCommandYaw);
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    processHardManueversXMode();
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

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
