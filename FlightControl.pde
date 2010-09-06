/*
  AeroQuad v2.0 - September 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)

void flightControl(void) {
  // ********************* Check Flight Mode *********************    
  if (flightMode == ACRO) {
    // Acrobatic Mode
    // updatePID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    // updatePID() is defined in PID.h
    motors.setMotorAxisCommand(ROLL, updatePID(receiver.getData(ROLL), gyro.getFlightData(ROLL) + 1500, &PID[ROLL]));
    motors.setMotorAxisCommand(PITCH, updatePID(receiver.getData(PITCH), gyro.getFlightData(PITCH) + 1500, &PID[PITCH]));
  }
  if (flightMode == STABLE) {
    // Stable Mode
    levelAdjust[ROLL] = updatePID(0, flightAngle.getData(ROLL), &PID[LEVELROLL]);
    levelAdjust[PITCH] = updatePID(0, -flightAngle.getData(PITCH), &PID[LEVELPITCH]);  
    motors.setMotorAxisCommand(ROLL, updatePID(receiver.getData(ROLL) + levelAdjust[ROLL], gyro.getFlightData(ROLL) + 1500, &PID[LEVELGYROROLL]));
    motors.setMotorAxisCommand(PITCH, updatePID(receiver.getData(PITCH) + levelAdjust[PITCH], gyro.getFlightData(PITCH) + 1500, &PID[LEVELGYROPITCH]));
  }
    
  // ***************************** Update Yaw ***************************
  // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
  // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
  if (headingHoldConfig == ON) {
    currentHeading += gyro.getData(YAW) * headingScaleFactor * G_Dt;
    if (receiver.getData(THROTTLE) > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
      if ((receiver.getData(YAW) > (MIDCOMMAND + 25)) || (receiver.getData(YAW) < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
        headingHold = 0;
        heading = currentHeading;
      }
      else // no yaw input, calculate current heading vs. desired heading heading hold
        headingHold = updatePID(heading, currentHeading, &PID[HEADING]);
    }
    else {
      heading = 0;
      currentHeading = 0;
      headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }   
  motors.setMotorAxisCommand(YAW, updatePID(receiver.getData(YAW) + headingHold, gyro.getFlightData(YAW) + 1500, &PID[YAW]));
    
  // *********************** Calculate Motor Commands **********************
  if (armed && safetyCheck) {
    #ifdef plusConfig
      motors.setMotorCommand(FRONT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
      motors.setMotorCommand(REAR, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
      motors.setMotorCommand(RIGHT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
      motors.setMotorCommand(LEFT, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    #endif
    #ifdef XConfig
      // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
      motors.setMotorCommand(FRONT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
      motors.setMotorCommand(RIGHT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
      motors.setMotorCommand(LEFT, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
      motors.setMotorCommand(REAR, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    #endif
    #ifdef MultipilotI2C
      // if using Mixertable need only Throttle MotorAxixCommand Roll,Pitch,Yaw Yet set
      motors.setThrottle(receiver.getData(THROTTLE));
    #endif
  }

  // ****************************** Altitude Adjust *************************
  // Experimental / not functional
  // s = (at^2)/2, t = 0.002
  //zAccelHover += ((accelData[ZAXIS] * accelScaleFactor) * 0.000004) * 0.5;
  /*zAccelHover = accelADC[ROLL] / tan(angleRad(ROLL));
  throttleAdjust = limitRange((zAccelHover - accelADC[ZAXIS]) * throttleAdjustGain, minThrottleAdjust, maxThrottleAdjust);
  for (motor = FRONT; motor < LASTMOTOR; motor++)
    motorCommand[motor] += throttleAdjust;*/

  // Prevents too little power applied to motors during hard manuevers
  // Also provides even motor power on both sides if limit encountered
  if ((motors.getMotorCommand(FRONT) <= MINTHROTTLE) || (motors.getMotorCommand(REAR) <= MINTHROTTLE)){
    delta = receiver.getData(THROTTLE) - MINTHROTTLE;
    motors.setMaxCommand(RIGHT, constrain(receiver.getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors.setMaxCommand(LEFT, constrain(receiver.getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((motors.getMotorCommand(FRONT) >= MAXCOMMAND) || (motors.getMotorCommand(REAR) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver.getData(THROTTLE);
    motors.setMinCommand(RIGHT, constrain(receiver.getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors.setMinCommand(LEFT, constrain(receiver.getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else {
    motors.setMaxCommand(RIGHT, MAXCOMMAND);
    motors.setMaxCommand(LEFT, MAXCOMMAND);
    motors.setMinCommand(RIGHT, MINTHROTTLE);
    motors.setMinCommand(LEFT, MINTHROTTLE);
  }

  if ((motors.getMotorCommand(LEFT) <= MINTHROTTLE) || (motors.getMotorCommand(RIGHT) <= MINTHROTTLE)){
    delta = receiver.getData(THROTTLE) - MINTHROTTLE;
    motors.setMaxCommand(FRONT, constrain(receiver.getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
    motors.setMaxCommand(REAR, constrain(receiver.getData(THROTTLE) + delta, MINTHROTTLE, MAXCHECK));
  }
  else if ((motors.getMotorCommand(LEFT) >= MAXCOMMAND) || (motors.getMotorCommand(RIGHT) >= MAXCOMMAND)) {
    delta = MAXCOMMAND - receiver.getData(THROTTLE);
    motors.setMinCommand(FRONT, constrain(receiver.getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
    motors.setMinCommand(REAR, constrain(receiver.getData(THROTTLE) - delta, MINTHROTTLE, MAXCOMMAND));
  }     
  else {
    motors.setMaxCommand(FRONT, MAXCOMMAND);
    motors.setMaxCommand(REAR, MAXCOMMAND);
    motors.setMinCommand(FRONT, MINTHROTTLE);
    motors.setMinCommand(REAR, MINTHROTTLE);
  }

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    #ifdef plusConfig
    if (receiver.getRaw(ROLL) < MINCHECK) {
      motors.setMinCommand(LEFT, minAcro);
      motors.setMaxCommand(RIGHT, MAXCOMMAND);
    }
    else if (receiver.getRaw(ROLL) > MAXCHECK) {
      motors.setMaxCommand(LEFT, MAXCOMMAND);
      motors.setMinCommand(RIGHT, minAcro);
    }
    else if (receiver.getRaw(PITCH) < MINCHECK) {
      motors.setMaxCommand(FRONT, MAXCOMMAND);
      motors.setMinCommand(REAR, minAcro);
    }
    else if (receiver.getRaw(PITCH) > MAXCHECK) {
      motors.setMinCommand(FRONT, minAcro);
      motors.setMaxCommand(REAR, MAXCOMMAND);
    }
    #endif
    #ifdef XConfig
    if (receiver.getRaw(ROLL) < MINCHECK) {
      motors.setMaxCommand(FRONT, minAcro);
      motors.setMaxCommand(REAR, MAXCOMMAND);
      motors.setMaxCommand(LEFT, minAcro);
      motors.setMaxCommand(RIGHT, MAXCOMMAND);
    }
    else if (receiver.getRaw(ROLL) > MAXCHECK) {
      motors.setMaxCommand(FRONT, MAXCOMMAND);
      motors.setMaxCommand(REAR, minAcro);
      motors.setMaxCommand(LEFT, MAXCOMMAND);
      motors.setMaxCommand(RIGHT, minAcro);
    }
    else if (receiver.getRaw(PITCH) < MINCHECK) {
      motors.setMaxCommand(FRONT, MAXCOMMAND);
      motors.setMaxCommand(REAR, minAcro);
      motors.setMaxCommand(LEFT, minAcro);
      motors.setMaxCommand(RIGHT, MAXCOMMAND);
    }
    else if (receiver.getRaw(PITCH) > MAXCHECK) {
      motors.setMaxCommand(FRONT, minAcro);
      motors.setMaxCommand(REAR, MAXCOMMAND);
      motors.setMaxCommand(LEFT, MAXCOMMAND);
      motors.setMaxCommand(RIGHT, minAcro);
    }
    #endif
  }

  // Apply limits to motor commands
  for (motor = FRONT; motor < LASTMOTOR; motor++)
    motors.setMotorCommand(motor, constrain(motors.getMotorCommand(motor), motors.getMinCommand(motor), motors.getMaxCommand(motor)));

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motors.setMotorCommand(motor, MINTHROTTLE);
  }
  // If motor output disarmed, force motor output to minimum
  if (armed == 0) {
    switch (calibrateESC) { // used for calibrating ESC's
    case 1:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMotorCommand(motor, MAXCOMMAND);
      break;
    case 3:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMotorCommand(motor, constrain(testCommand, 1000, 1200));
      break;
    case 5:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMotorCommand(motor, constrain(motors.getRemoteCommand(motor), 1000, 1200));
      safetyCheck = 1;
      break;
    default:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMotorCommand(motor, MINCOMMAND);
    }
    motors.write(); // Defined in Motors.h
  }

  // *********************** Command Motors **********************
 if (armed == 1 && safetyCheck == 1)
  motors.write(); // Defined in Motors.h
}
