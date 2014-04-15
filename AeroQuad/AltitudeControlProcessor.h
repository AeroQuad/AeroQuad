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

#ifndef MAX
  #define MAX(A,B) ((A)>(B) ? (A) : (B))
#endif

float targetVerticalSpeed = 0.0;
void processAltitudeHold()
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325
  // Now, thank to abarton for this complete improvement using accel z velicoty here
  // http://aeroquad.com/showthread.php?8912-Altitude-Hold-(again)

  if (altitudeHoldState == ON) {

    static float previousAltitudeToHoldTarget = 0.0;
    const float dT = 1.0/50.0;
    float altitudeToHoldTargetROC = (baroAltitudeToHoldTarget - previousAltitudeToHoldTarget) / dT;
    previousAltitudeToHoldTarget = baroAltitudeToHoldTarget;
    altitudeToHoldTargetROC = constrain(altitudeToHoldTargetROC, -1.0, 1.0);

    float vDeadBand = 0.125;
    float altitudeError = fabs(baroAltitudeToHoldTarget - estimatedAltitude);
    if (altitudeError > vDeadBand) {
      const float a = 5.0; // Deceleration rate as we approach target altitude, m/s^2.
      targetVerticalSpeed = a * sqrt(2.0 * (altitudeError-vDeadBand) / a);
      if (baroAltitudeToHoldTarget < estimatedAltitude) {
        targetVerticalSpeed = -targetVerticalSpeed;
      }
    } 
    else {
      targetVerticalSpeed = updatePIDDerivativeBaseRate(baroAltitudeToHoldTarget, estimatedAltitude, &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
    }
    targetVerticalSpeed += altitudeToHoldTargetROC;

    const float vMax = 5.0; // Maximum target speed, m/s.
    targetVerticalSpeed = constrain(targetVerticalSpeed, -vMax, vMax);

    float altitudeHoldThrottleCorrection = updatePIDDerivativeBaseRate(targetVerticalSpeed, zVelocity, &PID[ZDAMPENING_PID_IDX]);
    altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);

    if (abs(altitudeHoldThrottle - receiverCommand[receiverChannelMap[THROTTLE]]) > altitudeHoldPanicStickMovement) {
      altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
      altitudeHoldThrottleCorrection = 0;
    }
    else {

      float altitudeBump = 0.0;
      if (receiverCommand[receiverChannelMap[THROTTLE]] > (altitudeHoldThrottle + altitudeHoldBump)) {
        altitudeBump = ALTITUDE_BUMP_SPEED;
      }
      else if (receiverCommand[receiverChannelMap[THROTTLE]] < (altitudeHoldThrottle - altitudeHoldBump)) {
        altitudeBump = -ALTITUDE_BUMP_SPEED;
      }

      baroAltitudeToHoldTarget += altitudeBump;
    }
    throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection;

    //  Increase throttle to compensate for pitch or roll up to 45 degrees.
    throttle = throttle * (1.0 / MAX(0.707107, kinematicCorrectedAccel[ZAXIS]));
  }
  else {
    throttle = receiverCommand[receiverChannelMap[THROTTLE]];
  }
}



#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
