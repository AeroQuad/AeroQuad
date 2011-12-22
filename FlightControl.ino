/*
  AeroQuad v2.5.1 - December 2011
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#define ATTITUDE_SCALING (0.75 * PWM2RAD)
void calculateFlightError(void)
{
  if (flightMode == ACRO) {
    motors.setMotorAxisCommand(ROLL, updatePID(receiver.getSIData(ROLL), gyro.getData(ROLL), &PID[ROLL]));
    motors.setMotorAxisCommand(PITCH, updatePID(receiver.getSIData(PITCH), -gyro.getData(PITCH), &PID[PITCH]));
  }
  else {
    
  float rollAttitudeCmd = updatePID((receiver.getData(ROLL) - receiver.getZero(ROLL)) * ATTITUDE_SCALING, flightAngle->getData(ROLL), &PID[LEVELROLL]);
  float pitchAttitudeCmd = updatePID((receiver.getData(PITCH) - receiver.getZero(PITCH)) * ATTITUDE_SCALING, -flightAngle->getData(PITCH), &PID[LEVELPITCH]);
  motors.setMotorAxisCommand(ROLL, updatePID(rollAttitudeCmd, gyro.getData(ROLL), &PID[LEVELGYROROLL]));
  motors.setMotorAxisCommand(PITCH, updatePID(pitchAttitudeCmd, -gyro.getData(PITCH), &PID[LEVELGYROPITCH]));
  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processCalibrateESC(void)
{
  switch (calibrateESC) { // used for calibrating ESC's
  case 1:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      motors.setMotorCommand(motor, MAXCOMMAND);
    break;
  case 3:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      motors.setMotorCommand(motor, constrain(testCommand, 1000, 1200));
    break;
  case 5:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      motors.setMotorCommand(motor, constrain(motors.getRemoteCommand(motor), 1000, 1200));
    safetyCheck = ON;
    break;
  default:
    for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      motors.setMotorCommand(motor, MINCOMMAND);
  }
  // Send calibration commands to motors
  motors.write(); // Defined in Motors.h
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeadingHold ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading(void)
{
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      heading = degrees(flightAngle->getHeading(YAW));
    #else
      heading = degrees(gyro.getHeading());
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
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
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < .25 && relativeHeading > -.25) {
          headingHold = 0;
          PID[HEADING].integratedError = 0;
        }
        else if (headingHoldState == OFF) { // quick fix to soften heading hold on new heading
          if ((currentTime - headingTime) > 500000) {
            headingHoldState = ON;
            headingTime = currentTime;
            setHeading = heading;
            headingHold = 0;
          }
        }
        else {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
          headingHold = updatePID(0, relativeHeading, &PID[HEADING]);
          headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
        }
      }
    }
    else {
      // minimum throttle not reached, use off settings
      setHeading = heading;
      headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }
  // NEW SI Version
  commandedYaw = constrain(receiver.getSIData(YAW) + radians(headingHold), -PI, PI);
  motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, gyro.getData(YAW), &PID[YAW]));
  // uses flightAngle unbias rate
  //motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, flightAngle->getGyroUnbias(YAW), &PID[YAW]));
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processAltitudeHold //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processAltitudeHold(void)
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
#ifdef AltitudeHold
  if (altitudeHold == ON) {
    throttleAdjust = updatePID(holdAltitude, altitude.getData(), &PID[ALTITUDE]);
    //throttleAdjust = constrain((holdAltitude - altitude.getData()) * PID[ALTITUDE].P, minThrottleAdjust, maxThrottleAdjust);
    throttleAdjust = constrain(throttleAdjust, minThrottleAdjust, maxThrottleAdjust);
    if (abs(holdThrottle - receiver.getData(THROTTLE)) > PANICSTICK_MOVEMENT) {
      altitudeHold = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } else {
      if (receiver.getData(THROTTLE) > (holdThrottle + ALTBUMP)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        holdAltitude += 0.01;
      }
      if (receiver.getData(THROTTLE) < (holdThrottle - ALTBUMP)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        holdAltitude -= 0.01;
      }
    }
  }
  else {
    // Altitude hold is off, get throttle from receiver
    holdThrottle = receiver.getData(THROTTLE);
    throttleAdjust = autoDescent; // autoDescent is lowered from BatteryMonitor.h during battery alarm
  }
  // holdThrottle set in FlightCommand.pde if altitude hold is on
  throttle = holdThrottle + throttleAdjust; // holdThrottle is also adjust by BatteryMonitor.h during battery alarm
#else
  // If altitude hold not enabled in AeroQuad.pde, get throttle from receiver
  throttle = receiver.getData(THROTTLE) + autoDescent; //autoDescent is lowered from BatteryMonitor.h while battery critical, otherwise kept 0
#endif
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processMinMaxMotorCommand ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processMinMaxMotorCommand(void)
// JI - 11/25/11
// This is entirely new.  Based on MultiWii motor limiting,
// will work with any number of motors
{
  int maxMotor;
  
  // JI - 11/29/11 - Commented out the following lines as they are no longer necessary with the removal of 
  // JI - 11/29/11 - processHardManeuvers().  In flight motor command limits are now static at MINTHROTTLE
  // JI - 11/29/11 - and MAXCOMMAND
  //for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
  //{
  //  motors.setMinCommand(motor, MINTHROTTLE);
  //  motors.setMaxCommand(motor, MAXCOMMAND);
  //}

  maxMotor = motors.getMotorCommand(FIRSTMOTOR);
  
  for (byte motor=1; motor<LASTMOTOR; motor++)
    if (motors.getMotorCommand(motor) > maxMotor) maxMotor = motors.getMotorCommand(motor);
    
  for (byte motor = FIRSTMOTOR; motor < LASTMOTOR; motor++)
  {
    if (maxMotor > MAXCHECK)  
      motors.setMotorCommand(motor, (motors.getMotorCommand(motor) - (maxMotor - MAXCHECK)));
  }
  
  // JI - 11/29/11 - Moved the following 4 lines out of the individual configurations and moved them here for ease of maintenance
  // JI - 11/29/11 - Removed calls to motors.getMinCommand(motor) and motors.getMaxCommand(motor) since the maximum and minimum
  // JI - 11/29/11 - commands are no longer modified by processHardManeuvers
  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
    motors.setMotorCommand(motor, constrain(motors.getMotorCommand(motor), MINTHROTTLE, MAXCOMMAND));
  }

}

#ifdef XConfig
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// X MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlXMode(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
    motors.setMotorCommand(FRONT, throttle - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT, throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT, throttle + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR, throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif

#ifdef plusConfig
//////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlPlusMode(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors.setMotorCommand(FRONT, throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR, throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT, throttle - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT, throttle + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif

// JI - 11/25/11
// From this point down, new 8 motor motor mixes

#ifdef OCTOX_CONFIG
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// OCTOX CONFIG ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlOctoX(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    // FRONT  = Front/Left,  REAR  = Right/Rear,  LEFT  = Left/Rear,  RIGHT  = Front/Right 
    // FRONT2 = Front/Left2, REAR2 = Right/Rear2, LEFT2 = Left/Rear2, RIGHT2 = Front/Right2
    motors.setMotorCommand(FRONT,  throttle - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(FRONT2, motors.getMotorCommand(FRONT));
    motors.setMotorCommand(RIGHT,  throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT2, motors.getMotorCommand(RIGHT));
    motors.setMotorCommand(LEFT,   throttle + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT2,  motors.getMotorCommand(LEFT));
    motors.setMotorCommand(REAR,   throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR2,  motors.getMotorCommand(REAR));
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif

#ifdef X8PLUS_CONFIG
//////////////////////////////////////////////////////////////////////////////
////////////////////////////// X8 PLUS CONFIG ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlX8Plus(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors.setMotorCommand(FRONT,  throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR,   throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT,  throttle - motors.getMotorAxisCommand(ROLL)  + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT,   throttle + motors.getMotorAxisCommand(ROLL)  + motors.getMotorAxisCommand(YAW));

    motors.setMotorCommand(FRONT2, throttle - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR2,  throttle + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT2, throttle - motors.getMotorAxisCommand(ROLL)  - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT2,  throttle + motors.getMotorAxisCommand(ROLL)  - motors.getMotorAxisCommand(YAW));  
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif

#ifdef X8X_CONFIG
//////////////////////////////////////////////////////////////////////////////
////////////////////////////// X8 X CONFIG ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlX8X(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    // FRONT  = Upper Front/Left, REAR  = Upper Right/Rear, LEFT  = Upper Left/Rear, RIGHT  = Upper Front/Right 
    // FRONT2 = Lower Front/Left, REAR2 = Lower Right/Rear, LEFT2 = Lower Left/Rear, RIGHT2 = Lower Front/Right 
    motors.setMotorCommand(FRONT,  throttle - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT,  throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT,   throttle + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR,   throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    
    motors.setMotorCommand(FRONT2, throttle - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT2, throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT2,  throttle + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR2,  throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif

// JI - 12/13/11
// From this point down, new 6 motor motor mixes

#ifdef HEX_PLUS_CONFIG
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////HEX PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlHexPlus(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors.setMotorCommand(FRONT,  throttle                                    - 0.866025 * motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT,  throttle - motors.getMotorAxisCommand(ROLL) - 0.866025 * motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT2, throttle - motors.getMotorAxisCommand(ROLL) + 0.866025 * motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR,   throttle                                    + 0.866025 * motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT,   throttle + motors.getMotorAxisCommand(ROLL) + 0.866025 * motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT2,  throttle + motors.getMotorAxisCommand(ROLL) - 0.866025 * motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
} 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif

#ifdef HEX_X_CONFIG
//////////////////////////////////////////////////////////////////////////////
///////////////////////////// HEX PLUS MODE //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControlHexX(void) {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  //processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors.setMotorCommand(FRONT,  throttle + 0.866025 * motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(FRONT2, throttle - 0.866025 * motors.getMotorAxisCommand(ROLL) - motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT,  throttle - 0.866025 * motors.getMotorAxisCommand(ROLL)                                     - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR,   throttle - 0.866025 * motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(PITCH) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR2,  throttle + 0.866025 * motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT,   throttle + 0.866025 * motors.getMotorAxisCommand(ROLL)                                     + motors.getMotorAxisCommand(YAW));
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiver.getData(THROTTLE) < MINCHECK) {
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      motors.setMotorCommand(motor, MINTHROTTLE);
    }
  }

  // ESC Calibration
  if (armed == OFF) {
    processCalibrateESC();
  }

  // *********************** Command Motors **********************
  if (armed == ON && safetyCheck == ON) {
    motors.write(); // Defined in Motors.h
  }
}
#endif
