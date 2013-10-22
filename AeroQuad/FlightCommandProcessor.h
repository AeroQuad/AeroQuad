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

// If AUX1>MAXSWITCH then altitude hold is off
// If AUX1<MAXSWITCH and AUX1>MINSWITCH then altitude hold and position hold(if enabled) is on
// if AUX1<MINSWITCH then altitude hold and autopilot(if enabled) is on

// Need to figure out how to enable return to home
// Maybe AUX1 sets altitude hold, position hold and return to home
// Then AUX2 enables/disables autopilot, AUX1 return to home overrides autopilot

#if defined (AltitudeHoldBaro) || defined (AltitudeHoldRangeFinder)
  void processAltitudeHoldStateFromReceiverCommand() {
    if (receiverCommand[AUX1] < MAXSWITCH) {
      if (altitudeHoldState != ALTPANIC ) {  // check for special condition with mandatory override of Altitude hold
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
    if (receiverCommand[AUX3] < MAXSWITCH) {
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
      if (receiverCommand[AUX1] > SWITCHMAX) {
        altitudeHoldState = OFF;
        isAltitudeHoldInitialized = false;
      }
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


    if (receiverCommand[AUX1] > MAXSWITCH) {  // Enable autopilot
      if (!isGpsNavigationInitialized) {
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
        isGpsNavigationInitialized = true;
      }

      if (!isRouteInitialized) {
          loadNewRoute();
          isRouteInitialized = true;
      }

      positionHoldState = OFF;         // disable the position hold while navigating
      isPositionHoldInitialized = false;
  
      navigationState = ON;
    }
    else if ((receiverCommand[AUX1] > MINSWITCH) && (receiverCommand[AUX1] < MAXSWITCH)) {  // Enable position hold
      if (!isPositionHoldInitialized) {
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
  
        positionHoldPointToReach.latitude = currentPosition.latitude;
        positionHoldPointToReach.longitude = currentPosition.longitude;
        positionHoldPointToReach.altitude = getBaroAltitude();
        isPositionHoldInitialized = true;
        PID[GPSROLL_PID_IDX].integratedError = 0;
        PID[GPSPITCH_PID_IDX].integratedError = 0;
        PID[GPSYAW_PID_IDX].integratedError = 0;
      }
  
      isGpsNavigationInitialized = false;  // disable navigation
      isRouteInitialized = false;
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


void armMotors() {
  #ifdef OSD_SYSTEM_MENU
    if (menuOwnsSticks) {
      return;
    }
  #endif

  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorCommand[motor] = MINTHROTTLE;
  }
  motorArmed = ON;
  
  #ifdef EnableLogging
    logEnd();
    logInit();
    //logPrintF("throttle,adjThrottle,altHoldState,pressure,rawTemp,baroRawAlt,baroAlt\r\n)");
  #endif

  #ifdef OSD
    notifyOSD(OSD_CENTER|OSD_WARN, "!MOTORS ARMED!");
  #endif  

  zeroIntegralError();
}

void disarmMotors() {
  commandAllMotors(MINCOMMAND);
  motorArmed = OFF;
  inFlight = false;

  #ifdef EnableLogging
    logEnd();
  #endif

  #ifdef OSD
    notifyOSD(OSD_CENTER|OSD_WARN, "MOTORS UNARMED");
  #endif

  #if defined BattMonitorAutoDescent
    batteryMonitorAlarmCounter = 0;
    batteryMonitorStartThrottle = 0;
    batteyMonitorThrottleCorrection = 0.0;
  #endif
}

void zeroGyroAccel() {
  calibrateGyro();
  computeAccelBias();
  storeSensorsZeroToEEPROM();
  calibrateKinematics();
  zeroIntegralError();
  pulseMotors(3);
}

void processZeroThrottleFunctionFromReceiverCommand() {
  // Disarm motors (left stick lower left corner)
  #ifndef roverConfig
    if (receiverCommand[ZAXIS] < MINCHECK && motorArmed == ON) {
          disarmMotors();
    }
  #endif

  // Zero Gyro and Accel sensors (left stick lower left, right stick lower right corner)
  if ((receiverCommand[ZAXIS] < MINCHECK) && (receiverCommand[XAXIS] > MAXCHECK) && (receiverCommand[YAXIS] < MINCHECK)) {
	zeroGyroAccel();
  }   

  // Arm motors (left stick lower right corner)
  if (receiverCommand[ZAXIS] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) {
	armMotors();
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
  if (receiverCommand[MODE] > MIDCOMMAND) {
      flightMode = ATTITUDE_FLIGHT_MODE;
      simpleModeInitialize = false;
  }
  else {
      //flightMode = RATE_FLIGHT_MODE;
      flightMode = SIMPLE_FLIGHT_MODE;
  }

  if (previousFlightMode != flightMode) {
    zeroIntegralError();
    previousFlightMode = flightMode;
    if (flightMode == SIMPLE_FLIGHT_MODE)
        simpleModeInitialize = true;
  }

  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    //processAltitudeHoldStateFromReceiverCommand();
  #endif
  
  #if defined (AutoLanding)
    processAutoLandingStateFromReceiverCommand();
  #endif

  #if defined (UseGPSNavigator)
    processGpsNavigationStateFromReceiverCommand();
  #endif
}

#endif // _AQ_FLIGHT_COMMAND_READER_

