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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#define ATTITUDE_SCALING (0.75 * PWM2RAD)
void calculateFlightError(void)
{
  if (flightMode == ACRO) {
    motorAxisCommandRoll = updatePID(receiver->getSIData(ROLL), gyro->getRadPerSec(ROLL), &PID[ROLL]);
    motorAxisCommandPitch = updatePID(receiver->getSIData(PITCH), -gyro->getRadPerSec(PITCH), &PID[PITCH]);
  }
  else {
    
  float rollAttitudeCmd = updatePID((receiver->getData(ROLL) - receiver->getZero(ROLL)) * ATTITUDE_SCALING, kinematics->getData(ROLL), &PID[LEVELROLL]);
  float pitchAttitudeCmd = updatePID((receiver->getData(PITCH) - receiver->getZero(PITCH)) * ATTITUDE_SCALING, -kinematics->getData(PITCH), &PID[LEVELPITCH]);
  motorAxisCommandRoll = updatePID(rollAttitudeCmd, gyro->getRadPerSec(ROLL), &PID[LEVELGYROROLL]);
  motorAxisCommandPitch = updatePID(pitchAttitudeCmd, -gyro->getRadPerSec(PITCH), &PID[LEVELGYROPITCH]);
//  motors->setMotorAxisCommand(ROLL, updatePID(rollAttitudeCmd, flightAngle->getGyroUnbias(ROLL), &PID[LEVELGYROROLL]));
//  motors->setMotorAxisCommand(PITCH, updatePID(pitchAttitudeCmd, -flightAngle->getGyroUnbias(PITCH), &PID[LEVELGYROPITCH]));

  }
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processCalibrateESC //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processCalibrateESC(void)
{
  switch (calibrateESC) { // used for calibrating ESC's
  case 1:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motors->setMotorCommand(motor, MAXCOMMAND);
    break;
  case 3:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motors->setMotorCommand(motor, constrain(testCommand, 1000, 1200));
    break;
  case 5:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motors->setMotorCommand(motor, constrain(motorConfiguratorCommand[motor], 1000, 1200));
    safetyCheck = ON;
    break;
  default:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motors->setMotorCommand(motor, MINCOMMAND);
  }
  // Send calibration commands to motors
  motors->write(); // Defined in Motors.h
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processHeadingHold ///////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void processHeading(void)
{
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      heading = degrees(kinematics->getHeading(YAW));
    #else
      heading = degrees(gyro->getHeading());
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
    if (receiver->getData(THROTTLE) > MINCHECK ) { 
      if ((receiver->getData(YAW) > (MIDCOMMAND + 25)) || (receiver->getData(YAW) < (MIDCOMMAND - 25))) {
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
  commandedYaw = constrain(receiver->getSIData(YAW) + radians(headingHold), -PI, PI);
  motorAxisCommandYaw = updatePID(commandedYaw, gyro->getRadPerSec(YAW), &PID[YAW]);
  // uses flightAngle unbias rate
  //motors->setMotorAxisCommand(YAW, updatePID(commandedYaw, flightAngle->getGyroUnbias(YAW), &PID[YAW]));
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
    throttleAdjust = updatePID(holdAltitude, barometricSensor->getAltitude(), &PID[ALTITUDE]);
    //throttleAdjust = constrain((holdAltitude - altitude.getData()) * PID[ALTITUDE].P, minThrottleAdjust, maxThrottleAdjust);
    throttleAdjust = constrain(throttleAdjust, minThrottleAdjust, maxThrottleAdjust);
    if (abs(holdThrottle - receiver->getData(THROTTLE)) > PANICSTICK_MOVEMENT) {
      altitudeHold = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } else {
      if (receiver->getData(THROTTLE) > (holdThrottle + ALTBUMP)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        holdAltitude += 0.01;
      }
      if (receiver->getData(THROTTLE) < (holdThrottle - ALTBUMP)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        holdAltitude -= 0.01;
      }
    }
  }
  else {
    // Altitude hold is off, get throttle from receiver
    holdThrottle = receiver->getData(THROTTLE);
    throttleAdjust = autoDescent; // autoDescent is lowered from BatteryMonitor.h during battery alarm
  }
  // holdThrottle set in FlightCommand.pde if altitude hold is on
  throttle = holdThrottle + throttleAdjust; // holdThrottle is also adjust by BatteryMonitor.h during battery alarm
#else
  // If altitude hold not enabled in AeroQuad.pde, get throttle from receiver
  throttle = receiver->getData(THROTTLE) + autoDescent; //autoDescent is lowered from BatteryMonitor.h while battery critical, otherwise kept 0
#endif
}

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// processFlightControl main function ///////////////
//////////////////////////////////////////////////////////////////////////////
void processFlightControl() {
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();

  // ********************** Altitude Adjust **********************************
  processAltitudeHold();

  // ********************** Calculate Motor Commands *************************
  if (armed && safetyCheck) {
    applyMotorCommand();
  } 

  // *********************** process min max motor command *******************
  processMinMaxCommand();


  // Allows quad to do acrobatics by lowering power to opposite motors during hard manuevers
  processHardManuevers();

  
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

#endif //#define _AQ_PROCESS_FLIGHT_CONTROL_H_

