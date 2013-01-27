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

#ifndef _AQ_PID_H_
#define _AQ_PID_H_

enum {
  RATE_XAXIS_PID_IDX = 0,
  RATE_YAXIS_PID_IDX,
  ZAXIS_PID_IDX,
  ATTITUDE_XAXIS_PID_IDX,
  ATTITUDE_YAXIS_PID_IDX,
  HEADING_HOLD_PID_IDX,
  ATTITUDE_GYRO_XAXIS_PID_IDX,
  ATTITUDE_GYRO_YAXIS_PID_IDX,
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    BARO_ALTITUDE_HOLD_PID_IDX,
    ZDAMPENING_PID_IDX,
  #endif
  #if defined AltitudeHoldRangeFinder
    SONAR_ALTITUDE_HOLD_PID_IDX,
  #endif
  #if defined UseGPSNavigator
    GPSPITCH_PID_IDX,
    GPSROLL_PID_IDX,
    GPSYAW_PID_IDX,
  #endif    

  LAST_PID_IDX  // keep this definition at the end of this enum
};

//// PID Variables
struct PIDdata {
  float P, I, D;
  float lastError;
  // AKA experiments with PID
  float previousPIDTime;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
} PID[LAST_PID_IDX];

// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
float windupGuard; // Read in from EEPROM
//// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {

  // AKA PID experiments
  const float deltaPIDTime = (currentTime - PIDparameters->previousPIDTime) / 1000000.0;

  PIDparameters->previousPIDTime = currentTime;  // AKA PID experiments
  float error = targetPosition - currentPosition;

  if (inFlight) {
    PIDparameters->integratedError += error * deltaPIDTime;
  }
  else {
    PIDparameters->integratedError = 0.0;
  }
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -PIDparameters->windupGuard, PIDparameters->windupGuard);
  float dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastError) / (deltaPIDTime * 100); // dT fix from Honk
  PIDparameters->lastError = currentPosition;

  return (PIDparameters->P * error) + (PIDparameters->I * PIDparameters->integratedError) + dTerm;
}

void zeroIntegralError() __attribute__ ((noinline));
void zeroIntegralError() {
  for (byte axis = 0; axis <= ATTITUDE_YAXIS_PID_IDX; axis++) {
    PID[axis].integratedError = 0;
    PID[axis].previousPIDTime = currentTime;
  }
}

#endif // _AQ_PID_H_


