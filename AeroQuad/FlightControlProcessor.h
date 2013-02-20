/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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


#ifndef _AQ_PROCESS_FLIGHT_CONTROL_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_H_

#define ATTITUDE_SCALING (0.75 * PWM2RAD)


/**
 * calculateFlightError
 *
 * Calculate roll/pitch axis error with gyro/accel data to
 * compute motor command thrust so used command are executed
 */
void calculateFlightError()
{
  #if defined (UseGPSNavigator)
    if (navigationState == ON || positionHoldState == ON) {
      float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS] + gpsRollAxisCorrection) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
      float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS] + gpsPitchAxisCorrection) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
      motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
      motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
    }
    else
  #endif
  if (flightMode == ATTITUDE_FLIGHT_MODE) {
    float rollAttitudeCmd  = updatePID((receiverCommand[XAXIS] - receiverZero[XAXIS]) * ATTITUDE_SCALING, kinematicsAngle[XAXIS], &PID[ATTITUDE_XAXIS_PID_IDX]);
    float pitchAttitudeCmd = updatePID((receiverCommand[YAXIS] - receiverZero[YAXIS]) * ATTITUDE_SCALING, -kinematicsAngle[YAXIS], &PID[ATTITUDE_YAXIS_PID_IDX]);
    motorAxisCommandRoll   = updatePID(rollAttitudeCmd, gyroRate[XAXIS], &PID[ATTITUDE_GYRO_XAXIS_PID_IDX]);
    motorAxisCommandPitch  = updatePID(pitchAttitudeCmd, -gyroRate[YAXIS], &PID[ATTITUDE_GYRO_YAXIS_PID_IDX]);
  }
  else {
    motorAxisCommandRoll = updatePID(getReceiverSIData(XAXIS), gyroRate[XAXIS]*rotationSpeedFactor, &PID[RATE_XAXIS_PID_IDX]);
    motorAxisCommandPitch = updatePID(getReceiverSIData(YAXIS), -gyroRate[YAXIS]*rotationSpeedFactor, &PID[RATE_YAXIS_PID_IDX]);
  }
}

/**
 * processCalibrateESC
 * 
 * Proces esc calibration command with the help of the configurator
 */
void processCalibrateESC()
{
  switch (calibrateESC) { // used for calibrating ESC's
  case 1:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MAXCOMMAND;
    break;
  case 3:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(testCommand, 1000, 1200);
    break;
  case 5:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = constrain(motorConfiguratorCommand[motor], 1000, 1200);
    safetyCheck = ON;
    break;
  default:
    for (byte motor = 0; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }
  // Send calibration commands to motors
  writeMotors(); // Defined in Motors.h
}

/**
 * processBatteryMonitorThrottleAdjustment
 *
 * Check battery alarm and if in alarm, increment a counter
 * When this counter reach BATTERY_MONITOR_MAX_ALARM_COUNT, then
 * we are now in auto-descent mode.
 *
 * When in auto-descent mode, the user can pass throttle keep when the
 * alarm was reach, and the throttle is slowly decrease for a minute til
 * batteryMonitorThrottle that is configurable with the configurator
 */
#if defined BattMonitor && defined BattMonitorAutoDescent
  void processBatteryMonitorThrottleAdjustment() {
    
    if (batteryMonitorAlarmCounter < BATTERY_MONITOR_MAX_ALARM_COUNT) {
      if (batteryAlarm) {
        batteryMonitorAlarmCounter++;
      }
    }
    else {
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        if (altitudeHoldState == ON) {
          #if defined AltitudeHoldBaro
            baroAltitudeToHoldTarget -= 0.01;
          #endif
          #if defined AltitudeHoldRangeFinder
            if (sonarAltitudeToHoldTarget != INVALID_RANGE) {
              sonarAltitudeToHoldTarget -= 0.01;
            }
          #endif
        }
        else {
      #endif
          if (batteryMonitorStartThrottle == 0) {  // init battery monitor throttle correction!
            batteryMonitorStartTime = millis();
            if (throttle < batteryMonitorThrottleTarget) {
              batteryMonitorStartThrottle = batteryMonitorThrottleTarget;
            }
            else {
              batteryMonitorStartThrottle = throttle; 
            }
          }
          int batteryMonitorThrottle = map(millis()-batteryMonitorStartTime, 0, batteryMonitorGoingDownTime, batteryMonitorStartThrottle, batteryMonitorThrottleTarget);
          if (batteryMonitorThrottle < batteryMonitorThrottleTarget) {
            batteryMonitorThrottle = batteryMonitorThrottleTarget;
          }
          if (throttle < batteryMonitorThrottle) {
            batteyMonitorThrottleCorrection = 0;
          }
          else {
            batteyMonitorThrottleCorrection = batteryMonitorThrottle - throttle;
          }
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        }
      #endif
    }
  }
#endif  


#if defined AutoLanding
  #define BARO_AUTO_LANDING_DESCENT_SPEED 0.008
  #define SONAR_AUTO_LANDING_DESCENT_SPEED 0.005
  void processAutoLandingAltitudeCorrection() {
    if (autoLandingState != OFF) {   

      if (autoLandingState == BARO_AUTO_DESCENT_STATE) {
        baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
        if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) { 
          autoLandingState = SONAR_AUTO_DESCENT_STATE;
        }
      }
      else if (autoLandingState == SONAR_AUTO_DESCENT_STATE) {
        baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
        sonarAltitudeToHoldTarget -= SONAR_AUTO_LANDING_DESCENT_SPEED;
        if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] < 0.5) {
          autoLandingState = MOTOR_AUTO_DESCENT_STATE;
        }
      }
      else {
        autoLandingThrottleCorrection -= 1;
        baroAltitudeToHoldTarget -= BARO_AUTO_LANDING_DESCENT_SPEED;
        sonarAltitudeToHoldTarget -= SONAR_AUTO_LANDING_DESCENT_SPEED;

        if (((throttle + autoLandingThrottleCorrection) < 1000) || (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] < 0.20)) {
          commandAllMotors(MINCOMMAND);
          motorArmed = OFF;
        }
      }
    }
 }
#endif


/**
 * processThrottleCorrection
 * 
 * This function will add some throttle imput if the craft is angled
 * this prevent the craft to loose altitude when angled.
 * it also add the battery throttle correction in case
 * of we are in auto-descent.
 * 
 * Special thank to Ziojo for this.
 */
void processThrottleCorrection() {
 
  int throttleAdjust = 0;
  #if defined UseGPSNavigator
    if (navigationState == ON || positionHoldState == ON) {
      throttleAdjust = throttle / (cos (kinematicsAngle[XAXIS]*0.55) * cos (kinematicsAngle[YAXIS]*0.55));
      throttleAdjust = constrain ((throttleAdjust - throttle), 0, 50); //compensate max  +/- 25 deg XAXIS or YAXIS or  +/- 18 ( 18(XAXIS) + 18(YAXIS))
    }
  #endif
  #if defined BattMonitorAutoDescent
    throttleAdjust += batteyMonitorThrottleCorrection;
  #endif
  #if defined (AutoLanding)
    #if defined BattMonitorAutoDescent
      if (batteyMonitorThrottleCorrection != 0) { // don't auto land in the same time that the battery monitor do auto descent, or Override the auto descent to land, TBD
        throttleAdjust += autoLandingThrottleCorrection;
      }
    #else
      throttleAdjust += autoLandingThrottleCorrection;
    #endif
  #endif
  
  throttle = constrain((throttle + throttleAdjust),MINCOMMAND,MAXCOMMAND-150);  // limmit throttle to leave some space for motor correction in max throttle manuever
}


/**
 * processHardManuevers
 *
 * In case of a roll/pitch stick at one edge to do a loop, this function
 * will prevent the lower throttle motor side to have too much low throtte.
 */
void processHardManuevers() {
  
  if ((receiverCommand[XAXIS] < MINCHECK) ||
      (receiverCommand[XAXIS] > MAXCHECK) ||
      (receiverCommand[YAXIS] < MINCHECK) ||
      (receiverCommand[YAXIS] > MAXCHECK)) {  
        
    for (int motor = 0; motor < LASTMOTOR; motor++) {
      motorMinCommand[motor] = minArmedThrottle;
      motorMaxCommand[motor] = MAXCOMMAND;
    }
  }
}

/**
 * processMinMaxCommand
 *
 * This function correct too low/max throttle when manuevering
 * preventing some wobbling behavior
 */
void processMinMaxCommand()
{
  for (byte motor = 0; motor < LASTMOTOR; motor++)
  {
    motorMinCommand[motor] = minArmedThrottle;
    motorMaxCommand[motor] = MAXCOMMAND;
  }

  int maxMotor = motorCommand[0];
  
  for (byte motor=1; motor < LASTMOTOR; motor++) {
    if (motorCommand[motor] > maxMotor) {
      maxMotor = motorCommand[motor];
    }
  }
    
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    if (maxMotor > MAXCOMMAND) {
      motorCommand[motor] =  motorCommand[motor] - (maxMotor - MAXCOMMAND);
    }
  }
}

/**
 * processFlightControl
 *
 * Main flight control processos function
 */
void processFlightControl() {
  
  // ********************** Calculate Flight Error ***************************
  calculateFlightError();
  
  // ********************** Update Yaw ***************************************
  processHeading();
  
  if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  // 50hz task
    
    // ********************** Process position hold or navigation **************************
    #if defined (UseGPS)
      #if defined (UseGPSNavigator)
        processGpsNavigation();
      #endif  
    #endif
    
    // ********************** Process Altitude hold **************************
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      processAltitudeHold();
    #else
      throttle = receiverCommand[THROTTLE];
    #endif
    
    // ********************** Process Battery monitor hold **************************
    #if defined BattMonitor && defined BattMonitorAutoDescent
      processBatteryMonitorThrottleAdjustment();
    #endif

    // ********************** Process Auto-Descent  **************************
    #if defined AutoLanding
      processAutoLandingAltitudeCorrection();
    #endif
    
    // ********************** Process throttle correction ********************
    processThrottleCorrection();
  }

  // ********************** Calculate Motor Commands *************************
  if (motorArmed && safetyCheck) {
    applyMotorCommand();
  } 

  // *********************** process min max motor command *******************
  processMinMaxCommand();

  // If throttle in minimum position, don't apply yaw
  if (receiverCommand[THROTTLE] < MINCHECK) {
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motorMinCommand[motor] = minArmedThrottle;
      if (inFlight && flightMode == RATE_FLIGHT_MODE) {
        motorMaxCommand[motor] = MAXCOMMAND;
      }
      else {
        motorMaxCommand[motor] = minArmedThrottle;
      }
    }
  }
  
  // Apply limits to motor commands
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = constrain(motorCommand[motor], motorMinCommand[motor], motorMaxCommand[motor]);
  }

  // ESC Calibration
  if (motorArmed == OFF) {
    processCalibrateESC();
  }
  
  // *********************** Command Motors **********************
  if (motorArmed == ON && safetyCheck == ON) {
    writeMotors();
  }
}

#endif //#define _AQ_PROCESS_FLIGHT_CONTROL_H_

