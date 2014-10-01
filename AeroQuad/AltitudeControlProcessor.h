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

#define ALTITUDE_BUMP_SPEED 0.01




void processAltitudeHold() 
{
  if (receiverCommand[receiverChannelMap[THROTTLE]] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
    baroAltitudeToHoldTarget += ALTITUDE_BUMP_SPEED;
  }
  else if (receiverCommand[receiverChannelMap[THROTTLE]] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
    baroAltitudeToHoldTarget -= ALTITUDE_BUMP_SPEED;
  }

  float altitudeHoldThrottleCorrection = updatePID(baroAltitudeToHoldTarget, estimatedAltitude, &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
  altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, -400, 400);
  
  // ZDAMPENING COMPUTATIONS
  float zVelocityToReached = constrain(zVelocity, -altitudeHoldMaxVelocitySpeed, altitudeHoldMaxVelocitySpeed);
  const float zDampeningThrottleCorrection = updatePID(altitudeHoldThrottleCorrection, zVelocityToReached, &PID[ZDAMPENING_PID_IDX]);
  
  throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection + zDampeningThrottleCorrection;
}

void processVelocityHold()
{
  int userVelocityCommand = 0;
  if ((receiverCommand[receiverChannelMap[THROTTLE]] > (altitudeHoldThrottle + 15)) || 
      (receiverCommand[receiverChannelMap[THROTTLE]] < (altitudeHoldThrottle - 15))) {
    userVelocityCommand = (receiverCommand[receiverChannelMap[THROTTLE]] - altitudeHoldThrottle);
    userVelocityCommand = map(userVelocityCommand, -500, 500, -altitudeHoldMaxVelocitySpeed, altitudeHoldMaxVelocitySpeed);
  }
  userVelocityCommand = constrain(userVelocityCommand, -altitudeHoldMaxVelocitySpeed, altitudeHoldMaxVelocitySpeed);
  
  float zVelocityToReached = constrain(zVelocity, -altitudeHoldMaxVelocitySpeed, altitudeHoldMaxVelocitySpeed);
  const float zDampeningThrottleCorrection = updatePID(userVelocityCommand, zVelocityToReached, &PID[ZDAMPENING_PID_IDX]);
  
  throttle = altitudeHoldThrottle + zDampeningThrottleCorrection;
}

void processAltitudeControl()
{
  if (altitudeHoldState == ALTITUDE_HOLD_STATE) {
    processAltitudeHold();    
  }
  else if (altitudeHoldState == VELOCITY_HOLD_STATE){
    processVelocityHold();
  }
  else {
    throttle = receiverCommand[receiverChannelMap[THROTTLE]];
    if (throttle > 1850)
    {
      throttle = 1850;
    }
  }
}

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

