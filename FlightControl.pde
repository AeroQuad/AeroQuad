/*
  AeroQuad v2.1 - October 2010
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
  if (flightMode == ACRO) {
    // Acrobatic Mode
    // updatePID(target, measured, PIDsettings);
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    // updatePID() is defined in PID.h
    motors.setMotorAxisCommand(ROLL, updatePID(receiver.getData(ROLL), gyro.getFlightData(ROLL) + 1500, &PID[ROLL]));
    motors.setMotorAxisCommand(PITCH, updatePID(receiver.getData(PITCH), gyro.getFlightData(PITCH) + 1500, &PID[PITCH]));
    zeroIntegralError();
 }
 
  if (flightMode == STABLE) {
    // Stable Mode
    levelAdjust[ROLL] = (receiver.getAngle(ROLL) - flightAngle.getData(ROLL)) * PID[LEVELROLL].P;
    levelAdjust[PITCH] = (receiver.getAngle(PITCH) + flightAngle.getData(PITCH)) * PID[LEVELPITCH].P;
    // Check if pilot commands are not in hover, don't auto trim
    if ((abs(receiver.getTrimData(ROLL)) > levelOff) || (abs(receiver.getTrimData(PITCH)) > levelOff)) {
      zeroIntegralError();
      #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
        digitalWrite(LED2PIN, LOW);
      #endif
    }
    else {
      PID[LEVELROLL].integratedError = constrain(PID[LEVELROLL].integratedError + (((receiver.getAngle(ROLL) - flightAngle.getData(ROLL)) * G_Dt) * PID[LEVELROLL].I), -levelLimit, levelLimit);
      PID[LEVELPITCH].integratedError = constrain(PID[LEVELPITCH].integratedError + (((receiver.getAngle(PITCH) + flightAngle.getData(PITCH)) * G_Dt) * PID[LEVELROLL].I), -levelLimit, levelLimit);
      #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
        digitalWrite(LED2PIN, HIGH);
      #endif
    }
    motors.setMotorAxisCommand(ROLL, updatePID(receiver.getData(ROLL) + levelAdjust[ROLL], gyro.getFlightData(ROLL) + 1500, &PID[LEVELGYROROLL]) + PID[LEVELROLL].integratedError);
    motors.setMotorAxisCommand(PITCH, updatePID(receiver.getData(PITCH) + levelAdjust[PITCH], gyro.getFlightData(PITCH) + 1500, &PID[LEVELGYROPITCH]) + PID[LEVELPITCH].integratedError);
  }
    
  // ***************************** Update Yaw ***************************
  if (headingHoldConfig == ON) {
    gyro.calculateHeading();

    #ifdef HeadingMagHold
      heading = compass.getHeading();
    #else
      heading = gyro.getHeading();
    #endif
    
    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) relativeHeading += 360;
    if (heading >= (setHeading + 180)) relativeHeading -= 360;
    
    // Apply heading hold only when throttle high enough to start flight
    if (receiver.getData(THROTTLE) > MINCHECK ) { 
      if ((receiver.getData(YAW) > (MIDCOMMAND + 25)) || (receiver.getData(YAW) < (MIDCOMMAND - 25))) {
        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
      }
      else 
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
        headingHold = updatePID(0, relativeHeading, &PID[HEADING]);
    }
    else {
        // minimum throttle not reached, use off settings
        setHeading = heading;
        headingHold = 0;
        PID[HEADING].integratedError = 0;
    }
  }
  commandedYaw = constrain(receiver.getData(YAW) + headingHold, 1000, 2000);
  motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, gyro.getFlightData(YAW) + 1500, &PID[YAW]));
    
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
  #ifdef AltitudeHold
    if (altitudeHold == ON) {
      throttleAdjust = updatePID(holdAltitude, altitude.getData(), &PID[ALTITUDE]);
      zDampening = updatePID(0, accel.getZaxis(), &PID[ZDAMPENING]);
      throttleAdjust = constrain(throttleAdjust + zDampening, minThrottleAdjust, maxThrottleAdjust);
    }
    else
      throttleAdjust = 0;
  #endif

  // *********************** Calculate Motor Commands **********************
  if (armed && safetyCheck) {
    #ifdef plusConfig
      motors.setMotorCommand(FRONT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW) + throttleAdjust);
      motors.setMotorCommand(REAR, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW) + throttleAdjust);
      motors.setMotorCommand(RIGHT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW) + throttleAdjust);
      motors.setMotorCommand(LEFT, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW) + throttleAdjust);
    #endif
    #ifdef XConfig
      // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
      motors.setMotorCommand(FRONT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW) + throttleAdjust);
      motors.setMotorCommand(RIGHT, receiver.getData(THROTTLE) - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW) + throttleAdjust);
      motors.setMotorCommand(LEFT, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW) + throttleAdjust);
      motors.setMotorCommand(REAR, receiver.getData(THROTTLE) + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW) + throttleAdjust);
    #endif
    #ifdef MultipilotI2C
      // if using Mixertable need only Throttle MotorAxixCommand Roll,Pitch,Yaw Yet set
      motors.setThrottle(receiver.getData(THROTTLE));
    #endif
  } 

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
  
  // ESC Calibration
  if (armed == OFF) {
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
      safetyCheck = ON;
      break;
    default:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setMotorCommand(motor, MINCOMMAND);
    }
    // Send calibration commands to motors
    motors.write(); // Defined in Motors.h
  }

  // *********************** Command Motors **********************
 if (armed == ON && safetyCheck == ON)
  motors.write(); // Defined in Motors.h
}
