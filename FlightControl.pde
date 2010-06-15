/*
  AeroQuad v1.8 - June 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

void flightControl(void) {
  // ********************* Check Flight Mode *********************
    if (flightMode == ACRO) {
      // Acrobatic Mode
      // ************************** Update Roll/Pitch ***********************
      // updatePID(target, measured, PIDsettings);
      // measured = rate data from gyros scaled to PWM (1000-2000), since PID settings are found experimentally
      motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL], gyro.getData(ROLL) + 1500, &PID[ROLL]);
      motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH], gyro.getData(PITCH) + 1500, &PID[PITCH]);
    }
    if (flightMode == STABLE) {
      // Stable Mode
      // ************************** Update Roll/Pitch ***********************
      // updatePID(target, measured, PIDsettings);
      // measured = flight angle calculated from angle object
      // updatePID() and updatePIDangle() are defined in PID.h
      motorAxisCommand[ROLL] = updatePIDangle(transmitterCommandSmooth[ROLL] * mLevelTransmitter + bLevelTransmitter, flightAngle[ROLL], updatePID(0, gyro.getData(ROLL), &PID[LEVELGYROROLL]), &PID[LEVELROLL]);
      motorAxisCommand[PITCH] = updatePIDangle(transmitterCommandSmooth[PITCH] * mLevelTransmitter + bLevelTransmitter, -flightAngle[PITCH], updatePID(0, gyro.getData(PITCH), &PID[LEVELGYROPITCH]), &PID[LEVELPITCH]);
    }
    
  // ***************************** Update Yaw ***************************
  // Note: gyro tends to drift over time, this will be better implemented when determining heading with magnetometer
  // Current method of calculating heading with gyro does not give an absolute heading, but rather is just used relatively to get a number to lock heading when no yaw input applied
  if (headingHoldConfig == ON) {
    currentHeading += gyro.getData(YAW) * headingScaleFactor * controldT;
    if (transmitterCommand[THROTTLE] > MINCHECK ) { // apply heading hold only when throttle high enough to start flight
      if ((transmitterCommand[YAW] > (MIDCOMMAND + 25)) || (transmitterCommand[YAW] < (MIDCOMMAND - 25))) { // if commanding yaw, turn off heading hold
        headingHold = 0;
        heading = currentHeading;
      }
      else // no yaw input, calculate current heading vs. desired heading heading hold
        headingHold = updatePID(heading, currentHeading, &PID[HEADING]);
    }
    else {
      heading = 0;
      currentHeading = 0;
      headingHold = 0;
      PID[HEADING].integratedError = 0;
    }
  }   
  motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW] + headingHold, gyro.getData(YAW) + 1500, &PID[YAW]);
    
  // *********************** Calculate Motor Commands **********************
  if (armed && safetyCheck) {
    #ifdef plusConfig
      motorCommand[FRONT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[YAW];
      motorCommand[REAR] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[YAW];
      motorCommand[RIGHT] = transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] + motorAxisCommand[YAW];
      motorCommand[LEFT] = transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] + motorAxisCommand[YAW];
    #endif
    #ifdef XConfig
      // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
      motorCommand[FRONT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] + motorAxisCommand[ROLL] - motorAxisCommand[YAW];
      motorCommand[RIGHT] = transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[ROLL] + motorAxisCommand[YAW];
      motorCommand[LEFT] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[ROLL] + motorAxisCommand[YAW];
      motorCommand[REAR] = transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[ROLL] - motorAxisCommand[YAW];
    #endif
  }

  // ****************************** Altitude Adjust *************************
  // Experimental / not functional
  // s = (at^2)/2, t = 0.002
  //zAccelHover += ((accelData[ZAXIS] * accelScaleFactor) * 0.000004) * 0.5;
  /*zAccelHover = accelADC[ROLL] / tan(angleRad(ROLL));
  throttleAdjust = limitRange((zAccelHover - accelADC[ZAXIS]) * throttleAdjustGain, minThrottleAdjust, maxThrottleAdjust);
  for (motor = FRONT; motor < LASTMOTOR; motor++)
    motorCommand[motor] += throttleAdjust;*/

  // Prevents too little power applied to motors during hard manuevers
  // Also provides even motor power on both sides if limit encountered
  if ((motorCommand[FRONT] <= MINTHROTTLE) || (motorCommand[REAR] <= MINTHROTTLE)){
    delta = transmitterCommand[THROTTLE] - 1100;
    maxCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    maxCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[FRONT] >= MAXCOMMAND) || (motorCommand[REAR] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - transmitterCommand[THROTTLE];
    minCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    minCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    maxCommand[RIGHT] = MAXCOMMAND;
    maxCommand[LEFT] = MAXCOMMAND;
    minCommand[RIGHT] = MINTHROTTLE;
    minCommand[LEFT] = MINTHROTTLE;
  }

  if ((motorCommand[LEFT] <= MINTHROTTLE) || (motorCommand[RIGHT] <= MINTHROTTLE)){
    delta = transmitterCommand[THROTTLE] - 1100;
    maxCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
    maxCommand[REAR] = limitRange(transmitterCommand[THROTTLE] + delta, MINTHROTTLE, MAXCHECK);
  }
  else if ((motorCommand[LEFT] >= MAXCOMMAND) || (motorCommand[RIGHT] >= MAXCOMMAND)) {
    delta = MAXCOMMAND - transmitterCommand[THROTTLE];
    minCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
    minCommand[REAR] = limitRange(transmitterCommand[THROTTLE] - delta, MINTHROTTLE, MAXCOMMAND);
  }     
  else {
    maxCommand[FRONT] = MAXCOMMAND;
    maxCommand[REAR] = MAXCOMMAND;
    minCommand[FRONT] = MINTHROTTLE;
    minCommand[REAR] = MINTHROTTLE;
  }

  // Allows quad to do acrobatics by turning off opposite motors during hard manuevers
  if (flightMode == ACRO) {
    #ifdef plusConfig
    if (receiverData[ROLL] < MINCHECK) {
      minCommand[LEFT] = minAcro;
      maxCommand[RIGHT] = MAXCOMMAND;
    }
    else if (receiverData[ROLL] > MAXCHECK) {
      maxCommand[LEFT] = MAXCOMMAND;
      minCommand[RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {
     maxCommand[FRONT] = MAXCOMMAND;
     minCommand[REAR] = minAcro;
    }
    else if (receiverData[PITCH] > MAXCHECK) {
     minCommand[FRONT] = minAcro;
     maxCommand[REAR] = MAXCOMMAND;
    }
    #endif
    #ifdef XConfig
    if (receiverData[ROLL] < MINCHECK) {
      minCommand[FRONT] = minAcro;
      maxCommand[REAR] = MAXCOMMAND;
      minCommand[LEFT] = minAcro;
      maxCommand[RIGHT] = MAXCOMMAND;
    }
    else if (receiverData[ROLL] > MAXCHECK) {
      maxCommand[FRONT] = MAXCOMMAND;
      minCommand[REAR] = minAcro;
      maxCommand[LEFT] = MAXCOMMAND;
      minCommand[RIGHT] = minAcro;
    }
    else if (receiverData[PITCH] < MINCHECK) {
      maxCommand[FRONT] = MAXCOMMAND;
      minCommand[REAR] = minAcro;
      minCommand[LEFT] = minAcro;
      maxCommand[RIGHT] = MAXCOMMAND;
    }
    else if (receiverData[PITCH] > MAXCHECK) {
      minCommand[FRONT] = minAcro;
      maxCommand[REAR] = MAXCOMMAND;
      maxCommand[LEFT] = MAXCOMMAND;
      minCommand[RIGHT] = minAcro;
    }
    #endif
  }

  // Apply limits to motor commands
  for (motor = FRONT; motor < LASTMOTOR; motor++)
    motorCommand[motor] = limitRange(motorCommand[motor], minCommand[motor], maxCommand[motor]);

  // If throttle in minimum position, don't apply yaw
  if (transmitterCommand[THROTTLE] < MINCHECK) {
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINTHROTTLE;
  }
  // If motor output disarmed, force motor output to minimum
  if (armed == 0) {
    switch (calibrateESC) { // used for calibrating ESC's
    case 1:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motorCommand[motor] = MAXCOMMAND;
      break;
    case 3:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motorCommand[motor] = limitRange(testCommand, 1000, 1200);
      break;
    case 5:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motorCommand[motor] = limitRange(remoteCommand[motor], 1000, 1200);
      safetyCheck = 1;
      break;
    default:
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        motorCommand[motor] = MINCOMMAND;
    }
  }

  // *********************** Command Motors **********************
  motors.write(motorCommand); // Defined in Motors.h
}
