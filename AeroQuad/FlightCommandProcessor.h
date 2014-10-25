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




#if defined (AltitudeHoldBaro)
  boolean isAltitudeHoldEnabledByUser() {
    if (!(vehicleState & BARO_DETECTED)) {
      return false;
    }
    #if defined (UseGPS)
      if ((receiverCommand[receiverChannelMap[AUX1]] < 1666) || (receiverCommand[receiverChannelMap[AUX2]] < 1666)) {
        return true;
      }
      return false;
    #else
      if (receiverCommand[receiverChannelMap[AUX1]] < 1666) {
        return true;
      }
      return false;
    #endif
  }
  
  boolean isVelocityHoldStateEnabledByUser() {

    if (!(vehicleState & BARO_DETECTED)) {
      if (receiverCommand[receiverChannelMap[AUX1]] < 1666) {
        return true;
      }
      return false;
    }
    else if (receiverCommand[receiverChannelMap[AUX1]] > 1333 && receiverCommand[receiverChannelMap[AUX1]] < 1666 ) {
      return true;
    }
    return false;
  }
  
  void processAltitudeHoldStateFromReceiverCommand() {
    
    if (isVelocityHoldStateEnabledByUser()) {
      if (!isVelocityHoldInitialisez) {
        if (!inFlight) {
          altitudeHoldThrottle = MIDCOMMAND;
        }
        else {
          altitudeHoldThrottle = receiverCommand[receiverChannelMap[THROTTLE]];
        }
        isAltitudeHoldInitialized = false;
        isVelocityHoldInitialisez = true;
      }
      altitudeHoldState = VELOCITY_HOLD_STATE;
    } 
    else if (isAltitudeHoldEnabledByUser()) {
      if (!isAltitudeHoldInitialized) {
        baroAltitudeToHoldTarget = estimatedAltitude;
        PID[BARO_ALTITUDE_HOLD_PID_IDX].lastError = baroAltitudeToHoldTarget;
        if (!inFlight) {
          altitudeHoldThrottle = MIDCOMMAND;
        }
        else {
          altitudeHoldThrottle = receiverCommand[receiverChannelMap[THROTTLE]];
        }
        isAltitudeHoldInitialized = true;
        isVelocityHoldInitialisez = false;
      }
      altitudeHoldState = ALTITUDE_HOLD_STATE;
    }
    else {
      isAltitudeHoldInitialized = false;
      isVelocityHoldInitialisez = false;
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
          altitudeHoldThrottle = receiverCommand[receiverChannelMap[THROTTLE]];
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
        if ((receiverCommand[receiverChannelMap[AUX1]] > 1750) && (receiverCommand[receiverChannelMap[AUX2]] > 1750)) {
          altitudeHoldState = OFF;
          isAltitudeHoldInitialized = false;
        }
      #else
        if (receiverCommand[receiverChannelMap[AUX1]] > 1750) {
          altitudeHoldState = OFF;
          isAltitudeHoldInitialized = false;
        }
      #endif
    }
  }
#endif


#if defined (UseGPS)
  void processGpsNavigationStateFromReceiverCommand() {
    // Init home command
    if (motorArmed == OFF && 
        receiverCommand[receiverChannelMap[THROTTLE]] < MINCHECK && receiverCommand[receiverChannelMap[ZAXIS]] < MINCHECK &&
        receiverCommand[receiverChannelMap[YAXIS]] > MAXCHECK && receiverCommand[receiverChannelMap[XAXIS]] > MAXCHECK &&
        haveAGpsLock()) {
  
      homePosition.latitude = currentPosition.latitude;
      homePosition.longitude = currentPosition.longitude;
      homePosition.altitude = DEFAULT_HOME_ALTITUDE;
    }

    if (receiverCommand[receiverChannelMap[AUX2]] < 1750) {  // Enter in execute mission state, if none, go back home, override the position hold
      if (isInitNavigationNeeded) {
        
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;
        isInitNavigationNeeded = false;
      }

  
      positionHoldState = OFF;         // disable the position hold while navigating
      isStorePositionNeeded = true;
  
      navigationState = ON;
    }
    else if (receiverCommand[receiverChannelMap[AUX1]] < 1250) {  // Enter in position hold state
      if (isStorePositionNeeded) {
        
        gpsRollAxisCorrection = 0;
        gpsPitchAxisCorrection = 0;
        gpsYawAxisCorrection = 0;

        positionHoldPointToReach.latitude = currentPosition.latitude;
        positionHoldPointToReach.longitude = currentPosition.longitude;
        positionHoldPointToReach.altitude = getBaroAltitude();
        isStorePositionNeeded = false;
      }
  
      isInitNavigationNeeded = true;  // disable navigation
      navigationState = OFF;
  
      positionHoldState = ON;
    }
    else {
      // Navigation and position hold are disabled
      positionHoldState = OFF;
      isStorePositionNeeded = true;
  
      navigationState = OFF;
      isInitNavigationNeeded = true;
  
      gpsRollAxisCorrection = 0;
      gpsPitchAxisCorrection = 0;
      gpsYawAxisCorrection = 0;
    }
  }
#endif




void processZeroThrottleFunctionFromReceiverCommand() {
  // Disarm motors (left stick lower left corner)
  
  if (receiverCommand[receiverChannelMap[ZAXIS]] < MINCHECK && motorArmed == ON) {
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
  if ((receiverCommand[receiverChannelMap[ZAXIS]] < MINCHECK) && (receiverCommand[receiverChannelMap[XAXIS]] > MAXCHECK) && (receiverCommand[receiverChannelMap[YAXIS]] < MINCHECK)) {
    calibrateGyro();
    computeAccelBias();
    storeSensorsZeroToEEPROM();
    zeroIntegralError();
    pulseMotors(3);
  }   

  // Arm motors (left stick lower right corner)
  if (receiverCommand[receiverChannelMap[ZAXIS]] > MAXCHECK && motorArmed == OFF && safetyCheck == ON) {

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
  if (receiverCommand[receiverChannelMap[ZAXIS]] > MINCHECK) {
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
  
  if (receiverCommand[receiverChannelMap[THROTTLE]] < MINCHECK) {
    processZeroThrottleFunctionFromReceiverCommand();
  }

  if (!inFlight) {
    if (motorArmed == ON && receiverCommand[receiverChannelMap[THROTTLE]] > minArmedThrottle) {
      inFlight = true;
    }
  }

  // Check Mode switch for Acro or Stable
  #if defined USE_HORIZON_MODE
    if (receiverCommand[receiverChannelMap[MODE]] > 1666) {
      flightMode = ATTITUDE_FLIGHT_MODE;
    }
    else if (receiverCommand[receiverChannelMap[MODE]] < 1666 && receiverCommand[receiverChannelMap[MODE]] > 1333) {
      flightMode = HORIZON_FLIGHT_MODE;
    }
    else {
      flightMode = RATE_FLIGHT_MODE;
    }
  #else
    if (receiverCommand[receiverChannelMap[MODE]] > 1666) {
      flightMode = ATTITUDE_FLIGHT_MODE;
    }
    else {
      flightMode = RATE_FLIGHT_MODE;
    }
  #endif
  
  if (previousFlightMode != flightMode) {
    zeroIntegralError();
  }
  previousFlightMode = flightMode;
    
  #if defined AltitudeHoldBaro
    processAltitudeHoldStateFromReceiverCommand();
  #endif
  
  #if defined (AutoLanding)
    processAutoLandingStateFromReceiverCommand();
  #endif

  #if defined (UseGPS)
    if (isGpsEnabled) {
      processGpsNavigationStateFromReceiverCommand();
    }
    else {
      positionHoldState = OFF;
      navigationState = OFF;
    }
  #endif
}

#endif // _AQ_FLIGHT_COMMAND_READER_

