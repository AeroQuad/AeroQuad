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
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////




#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_


#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

#define INVALID_THROTTLE_CORRECTION -1000
#define ALTITUDE_BUMP_SPEED 0.01

float previousZDampeningThrottleCorrection = 0.0;


void processAltitudeHold() 
{
  if (receiverCommand[receiverChannelMap[THROTTLE]] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
    baroAltitudeToHoldTarget += ALTITUDE_BUMP_SPEED;
  }
  else if (receiverCommand[receiverChannelMap[THROTTLE]] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
    baroAltitudeToHoldTarget -= ALTITUDE_BUMP_SPEED;
  }

  float altitudeHoldThrottleCorrection = updatePID(baroAltitudeToHoldTarget, estimatedAltitude, &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
  altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
  
  // ZDAMPENING COMPUTATIONS
  float zDampeningThrottleCorrection = updatePID(altitudeHoldThrottleCorrection, zVelocity, &PID[ZDAMPENING_PID_IDX]);
  zDampeningThrottleCorrection = constrain(zDampeningThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);      
  
  throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection + zDampeningThrottleCorrection;
}

void processVelocityHold()
{
  int userVelocityCommand = (receiverCommand[receiverChannelMap[THROTTLE]] - altitudeHoldThrottle) / 2;
  userVelocityCommand = constrain(userVelocityCommand, -100, 100);
  
  float zDampeningThrottleCorrection = updatePID(userVelocityCommand, zVelocity, &PID[ZDAMPENING_PID_IDX]);
  zDampeningThrottleCorrection = constrain(zDampeningThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);      
  
  throttle = altitudeHoldThrottle + zDampeningThrottleCorrection;
}

void processAltitudeControl()
{
  if (altitudeHoldState == ALTPANIC) {
    return;
  }
  
  if (altitudeHoldState == ALTITUDE_HOLD_STATE || altitudeHoldState == ALTITUDE_HOLD_STATE) {
    if (abs(altitudeHoldThrottle - receiverCommand[receiverChannelMap[THROTTLE]]) > altitudeHoldPanicStickMovement) {
      altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
      return;
    }
  }

  if (altitudeHoldState == ALTITUDE_HOLD_STATE) {
    processAltitudeHold();    
  }
  else if (altitudeHoldState == VELOCITY_HOLD_STATE){
    processVelocityHold();
  }
  else {
    throttle = receiverCommand[receiverChannelMap[THROTTLE]];
  }
}

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

