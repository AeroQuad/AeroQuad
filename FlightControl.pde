/*
  AeroQuad v2.3 - March 2011
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
// Special thanks to Keny9999 for suggesting a more readable format for FlightControl.pde and for
// porting over the ArduPirates Stable Mode (please note this is still experimental, use at your own risk)



//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Attitude Mode ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// To Do
// Figure out how to zero integrator when entering attitude mode from rate mode 
// 2.3 Original
float attitudeScaling = (1.0 * PWM2RPS); // +/-1.0 radian attitude
// 2.3 Stable
//float attitudeScaling = (1.5 * PWM2RPS); // +/-1.5 radian attitude factored further by transmitter factor
void processAttitudeMode(void)
{
  float rollAttitudeCmd = updatePID((receiver.getData(ROLL) - receiver.getZero(ROLL)) * attitudeScaling, flightAngle->getData(ROLL), &PID[LEVELROLL]);
  float pitchAttitudeCmd = updatePID((receiver.getData(PITCH) - receiver.getZero(PITCH)) * attitudeScaling, -flightAngle->getData(PITCH), &PID[LEVELPITCH]);
  motors.setMotorAxisCommand(ROLL, updatePID(rollAttitudeCmd, flightAngle->getGyroUnbias(ROLL), &PID[LEVELGYROROLL]));
  motors.setMotorAxisCommand(PITCH, updatePID(pitchAttitudeCmd, -flightAngle->getGyroUnbias(PITCH), &PID[LEVELGYROPITCH]));
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void calculateFlightError(void)
{
  if (flightMode == ACRO) {
    // Acrobatic Mode
    // updatePID(target, measured, PIDsettings);
    // updatePID() is defined in PID.h
    // NEW SI Version
    // measured = rate data from gyros scaled to Radians (-1.5*PI/+1.5*PI), since PID settings are found experimentally
    motors.setMotorAxisCommand(ROLL, updatePID(receiver.getSIData(ROLL), flightAngle->getGyroUnbias(ROLL), &PID[ROLL]));
    motors.setMotorAxisCommand(PITCH, updatePID(receiver.getSIData(PITCH), -flightAngle->getGyroUnbias(PITCH), &PID[PITCH]));
    // NEW SI Version uses DCM unbias rate
    //motors.setMotorAxisCommand(ROLL, updatePID(receiver.getSIData(ROLL), flightAngle->getGyroUnbias(ROLL), &PID[ROLL]));
    //motors.setMotorAxisCommand(PITCH, updatePID(receiver.getSIData(PITCH), -flightAngle->getGyroUnbias(PITCH), &PID[PITCH]));
    // OLD NON SI
    // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
    //motors.setMotorAxisCommand(ROLL, updatePID(receiver.getData(ROLL), gyro.getFlightData(ROLL) + 1500, &PID[ROLL]));
    //motors.setMotorAxisCommand(PITCH, updatePID(receiver.getData(PITCH), gyro.getFlightData(PITCH) + 1500, &PID[PITCH]));
    zeroIntegralError();
  }
  else {
    processAttitudeMode();
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
    //gyro.calculateHeading();

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
      }
      else {
        if (relativeHeading < .25 && relativeHeading > -.25) {
          //setHeading = heading;
          headingHold = 0;
          PID[HEADING].integratedError = 0;
        }
        else {
        // No new yaw input, calculate current heading vs. desired heading heading hold
        // Relative heading is always centered around zero
          headingHold = updatePID(0, relativeHeading, &PID[HEADING]);
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
  //motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, flightAngle->getGyroUnbias(YAW), &PID[YAW]));
  // OLD NON SI
  //commandedYaw = constrain(receiver.getData(YAW) + headingHold, 1000, 2000);
  //motors.setMotorAxisCommand(YAW, updatePID(commandedYaw, gyro.getFlightData(YAW) + 1500, &PID[YAW]));
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
    //zDampening = updatePID(0, accel.getZaxis(), &PID[ZDAMPENING]); // This is stil under development - do not use (set PID=0)
    //if((abs(flightAngle->getData(ROLL)) > radians(5)) || (abs(flightAngle->getData(PITCH)) > radians(5))) { 
    //  PID[ZDAMPENING].integratedError = 0;
    //}
    //throttleAdjust = constrain((holdAltitude - altitude.getData()) * PID[ALTITUDE].P, minThrottleAdjust, maxThrottleAdjust);
    throttleAdjust = constrain(throttleAdjust, minThrottleAdjust, maxThrottleAdjust);
    if (abs(holdThrottle - receiver.getData(THROTTLE)) > PANICSTICK_MOVEMENT) {
      altitudeHold = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } else {
    #ifdef BinaryWrite
    #ifdef OpenlogBinaryWrite
      // change this to the new BinaryWrite
      //SerialLog.dumpRecord(LOG_REC_FLIGHT);
      //SerialLog.dumpRecord(LOG_REC_ALTHOLD);
      //SerialLog.dumpRecord(LOG_REC_ALTPID);
    #endif      
    #endif
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
  //zDampening = updatePID(0, accel.getZaxis(), &PID[ZDAMPENING]); // This is stil under development - do not use (set PID=0)
  //throttle = receiver.getData(THROTTLE) - zDampening + autoDescent; 
  // If altitude hold not enabled in AeroQuad.pde, get throttle from receiver
  throttle = receiver.getData(THROTTLE) + autoDescent; //autoDescent is lowered from BatteryMonitor.h while battery critical, otherwise kept 0
#endif
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processMinMaxMotorCommand ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processMinMaxMotorCommand(void)
{
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
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// processHardManuevers ////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHardManuevers()
{
#ifdef XConfig    // Fix for + mode hardmanuevers
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
  processAltitudeHold();

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

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    processHardManuevers();
  }

  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
    motors.setMotorCommand(motor, constrain(motors.getMotorCommand(motor), motors.getMinCommand(motor), motors.getMaxCommand(motor)));
  }

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
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    motors.setMotorCommand(FRONT, throttle - motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(REAR, throttle + motors.getMotorAxisCommand(PITCH) - motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(RIGHT, throttle - motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
    motors.setMotorCommand(LEFT, throttle + motors.getMotorAxisCommand(ROLL) + motors.getMotorAxisCommand(YAW));
  } 

  // *********************** process min max motor command *******************
  processMinMaxMotorCommand();

  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  if (flightMode == ACRO) {
    processHardManuevers();
  }

  // Apply limits to motor commands
  for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
    motors.setMotorCommand(motor, constrain(motors.getMotorCommand(motor), motors.getMinCommand(motor), motors.getMaxCommand(motor)));
  }

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

