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

// FlightCommandProcessor is responsible for decoding transmitter stick combinations
// for setting up AeroQuad modes such as motor arming and disarming

#ifndef _AQ_FLIGHT_COMMAND_READER_
#define _AQ_FLIGHT_COMMAND_READER_




#if defined (AltitudeHoldBaro) || defined (AltitudeHoldRangeFinder)
  boolean isPositionHoldEnabledByUser() {
    #if defined (UseGPSNavigator)
      if ((receiverCommand[AUX1] < 1750) || (receiverCommand[AUX2] < 1750)) {
        return true;
      }
      return false;
    #else
      if (receiverCommand[AUX1] < 1750) {
        return true;
      }
      return false;
    #endif
  }
#endif

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  void processAltitudeHoldStateFromReceiverCommand() {
    if (isPositionHoldEnabledByUser()) {
      if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
        if (!isAltitudeHoldInitialized) {
          #if defined AltitudeHoldBaro
            baroAltitudeToHoldTarget = getBaroAltitude();
            PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
          #endif
          #if defined AltitudeHoldRangeFinder
            sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
            PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
          #endif
          altitudeHoldThrottle = receiverCommand[THROTTLE];
          isAltitudeHoldInitialized = true;
        }
        altitudeHoldState = ON;
      }
    } 
    else {
      isAltitudeHoldInitialized = false;
      altitudeHoldState = OFF;
    }
  }
#endif


#if defined (AutoLanding)
  void processAutoLandingStateFromReceiverCommand() {
    if (receiverCommand[AUX3] < 1750) {
      if (altitudeHoldState != ALTPANIC ) {  // check for special condition with manditory override of Altitude hold
        if (isAutoLandingInitialized) {
          autoLandingState = BARO_AUTO_DESCENT_STATE;
          #if defined AltitudeHoldBaro
            baroAltitudeToHoldTarget = getBaroAltitude();
            PID[BARO_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
          #endif
          #if defined AltitudeHoldRangeFinder
            sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
            PID[SONAR_ALTITUDE_HOLD_PID_IDX].integratedError = 0;
            PID[SONAR_ALTITUDE_HOLD_PID_IDX].lastError = sonarAltitudeToHoldTarget;
          #endif
          altitudeHoldThrottle = receiverCommand[THROTTLE];
          isAutoLandingInitialized = true;
        }
        altitudeHoldState = ON;
      }
    }
    else {
      autoLandingState = OFF;
      autoLandingThrottleCorrection = 0;
      isAutoLandingInitialized = false;
      #if defined (UseGPSNavigator)
        if ((receiverCommand[AUX1] > 1750) && (receiverCommand[AUX2] > 1750)) {
          altitudeHoldState = OFF;
          isAltitudeHoldInitialized = false;
        }
      #else
        if (receiverCommand[AUX1] > 1750) {
          altitudeHoldState = OFF;
          isAltitudeHoldInitialized = false;
        }
      #endif
    }
  }
#endif


#if defined (UseGPSNavigator)
  void processGpsNavigationStateFromReceiverCommand() {
    // Init home command
    if (motorArmed == OFF && 
        receiverCommand[THROTTLE] < MINCHECK && receiverCommand[ZAXIS] < MINCHECK &&
        receiverCommand[YAXIS] > MAXCHECK && receiverCommand[XAXIS] > MAXCHECK &&
        haveAGpsLock()) {
  
      homePosition.latitude = currentPosition.latitude;
      homePosition.longitude = currentPosition.longitude;
      homePosition.altitude = DEFAULT_HOME_ALTITUDE;
    }


    if (receiverCommand[AUX2] < 1750) {  // Enter in execute mission state, if none, go back home, override the position hold
      if (!isGpsNavigationInitialized) {
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
        isGpsNavigationInitialized = true;
      }
  
      positionHoldState = OFF;         // disable the position hold while navigating
      isPositionHoldInitialized = false;
  
      navigationState = ON;
    }
    else if (receiverCommand[AUX1] < 1250) {  // Enter in position hold state
      if (!isPositionHoldInitialized) {
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
  
        positionHoldPointToReach.latitude = currentPosition.latitude;
        positionHoldPointToReach.longitude = currentPosition.longitude;
        positionHoldPointToReach.altitude = getBaroAltitude();
        isPositionHoldInitialized = true;
      }
  
      isGpsNavigationInitialized = false;  // disable navigation
      navigationState = OFF;
  
      positionHoldState = ON;
    }
    else {
      // Navigation and position hold are disabled
      positionHoldState = OFF;
      isPositionHoldInitialized = false;
  
      navigationState = OFF;
      isGpsNavigationInitialized = false;
  
      gpsRollAxisCorrection = 0;
      gpsPitchAxisCorrection = 0;
      gpsYawAxisCorrection = 0;
    }
  }
#endif




void processZeroThrottleFunctionFromReceiverCommand() {
  // Disarm motors (left stick lower left corner)
  if (receiverCommand[ZAXIS] < MINCHECK && motorArmed == ON) {
    commandAllMotors(MINCOMMAND);
    motorArmed = OFF;
    inFlight = false;

    #ifdef OSD
      notifyOSD(OSD_CENTER|OSD_WARN, "MOTORS UNARMED");
    #endif

    #if defined BattMonitorAutoDescent
      batteryMonitorAlarmCounter = 0;
      batteryMonitorStartThrottle = 0;
      batteyMonitorThrottleCorrection = 0.0;
    #endif
  }    

  // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
  if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) {
    calibrateGyro();
    computeAccelBias();
    storeSensorsZeroToEEPROM();
    calibrateKinematics();
    zeroIntegralError();
    pulseMotors(3);
  }   

  // Arm motors (left stick lower right corner)
  if (receiverCommand[ZAXIS] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) {

    #ifdef OSD_SYSTEM_MENU
      if (menuOwnsSticks) {
        return;
      }
    #endif

    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      motorCommand[motor] = MINTHROTTLE;
    }
    motorArmed = ON;

    #ifdef OSD
      notifyOSD(OSD_CENTER|OSD_WARN, "!MOTORS ARMED!");
    #endif  

    zeroIntegralError();

  }
  // Prevents accidental arming of motor output if no transmitter command received
  if (receiverCommand[ZAXIS] > MINCHECK) {
    safetyCheck = ON; 
  }
}




/**
 * readPilotCommands
 * 
 * This function is responsible to read receiver
 * and process command from the users
 */
void readPilotCommands() {

  readReceiver(); 
  
  if (receiverCommand[THROTTLE] < MINCHECK) {
    processZeroThrottleFunctionFromReceiverCommand();
  }

  if (!inFlight) {
    if (motorArmed == ON && receiverCommand[THROTTLE] > minArmedThrottle) {
      inFlight = true;
    }
  }

  // Check Mode switch for Acro or Stable
  if (receiverCommand[MODE] > 1500) {
    flightMode = ATTITUDE_FLIGHT_MODE;
  }
  else {
    flightMode = RATE_FLIGHT_MODE;
  }


  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    processAltitudeHoldStateFromReceiverCommand();
  #endif
  
  #if defined (AutoLanding)
    processAutoLandingStateFromReceiverCommand();
  #endif

  #if defined (UseGPSNavigator)
    processGpsNavigationStateFromReceiverCommand();
  #endif
}

#endif // _AQ_FLIGHT_COMMAND_READER_

