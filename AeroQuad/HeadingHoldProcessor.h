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


#ifndef _AQ_HEADING_CONTROL_PROCESSOR_H_
#define _AQ_HEADING_CONTROL_PROCESSOR_H_


float setHeading          = 0;
unsigned long headingTime = micros();


/**
 * processHeading
 *
 * This function will calculate the craft heading correction depending 
 * of the users command. Heading correction is process with the gyro
 * or a magnetometer
 */
void processHeading()
{
  if (headingHoldConfig == ON) {

    #if defined(HeadingMagHold)
      heading = degrees(trueNorthHeading);
    #else
      heading = degrees(gyroHeading);
    #endif

    // Always center relative heading around absolute heading chosen during yaw command
    // This assumes that an incorrect yaw can't be forced on the AeroQuad >180 or <-180 degrees
    // This is done so that AeroQuad does not accidentally hit transition between 0 and 360 or -180 and 180
    // AKA - THERE IS A BUG HERE - if relative heading is greater than 180 degrees, the PID will swing from negative to positive
    // Doubt that will happen as it would have to be uncommanded.
    relativeHeading = heading - setHeading;
    if (heading <= (setHeading - 180)) {
      relativeHeading += 360;
    }
    if (heading >= (setHeading + 180)) {
      relativeHeading -= 360;
    }

    // Apply heading hold only when throttle high enough to start flight
    if (receiverCommand[THROTTLE] > MINCHECK ) { 
      
    #if defined (UseGPSNavigator)
      if (( (receiverCommand[ZAXIS] + gpsYawAxisCorrection) > (MIDCOMMAND + 25)) ||
          ( (receiverCommand[ZAXIS] + gpsYawAxisCorrection) < (MIDCOMMAND - 25))) {
    #else
      if ((receiverCommand[ZAXIS] > (MIDCOMMAND + 25)) ||
          (receiverCommand[ZAXIS] < (MIDCOMMAND - 25))) {
    #endif

        // If commanding yaw, turn off heading hold and store latest heading
        setHeading = heading;
        headingHold = 0;
        PID[HEADING_HOLD_PID_IDX].integratedError = 0;
        headingHoldState = OFF;
        headingTime = currentTime;
      }
      else {
        if (relativeHeading < 0.25 && relativeHeading > -0.25) {
          headingHold = 0;
          PID[HEADING_HOLD_PID_IDX].integratedError = 0;
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
          #if defined (UseGPSNavigator)
            if (navigationState == ON) {
              headingHold = updatePID(groundTrackHeading, heading, &PID[HEADING_HOLD_PID_IDX]);
            }
            else {
              headingHold = updatePID(0, relativeHeading, &PID[HEADING_HOLD_PID_IDX]);
              headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
            }
          #else
            // No new yaw input, calculate current heading vs. desired heading heading hold
            // Relative heading is always centered around zero
            headingHold = updatePID(0, relativeHeading, &PID[HEADING_HOLD_PID_IDX]);
            headingTime = currentTime; // quick fix to soften heading hold, wait 100ms before applying heading hold
          #endif
        }
      }
    }
    else {
      // minimum throttle not reached, use off settings
      setHeading = heading;
      headingHold = 0;
      PID[HEADING_HOLD_PID_IDX].integratedError = 0;
    }
  }

  // NEW SI Version
  float receiverSiData = (receiverCommand[ZAXIS] - receiverZero[ZAXIS]) * (2.5 * PWM2RAD);
  const float commandedYaw = constrain(receiverSiData + radians(headingHold), -PI, PI);
  motorAxisCommandYaw = updatePID(commandedYaw, gyroRate[ZAXIS], &PID[ZAXIS_PID_IDX]);
}

#endif






