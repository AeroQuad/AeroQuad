/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

/*
       CW  0....Front....0 CCW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CW
*/

#include "FlightControlVariable.h"

#ifdef OLD_MOTOR_NUMBERING  
  #define FRONT_LEFT  MOTOR1
  #define REAR_RIGHT  MOTOR2
  #define FRONT_RIGHT MOTOR3
  #define REAR_LEFT   MOTOR4
#else
  #define FRONT_LEFT  MOTOR1
  #define FRONT_RIGHT MOTOR2
  #define REAR_RIGHT  MOTOR3
  #define REAR_LEFT   MOTOR4
#endif
#define LASTMOTOR   MOTOR4+1

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

byte maxLimit = OFF;
byte minLimit = OFF;
int motorMaxLimitCommand[4] = {0,0,0,0};
int motorMinLimitCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int correctedThrottle = throttle - abs(motorAxisCommandYaw*2/4);
  motorCommand[FRONT_LEFT] = correctedThrottle - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = correctedThrottle - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT] = correctedThrottle + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT] = correctedThrottle + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);

  // Force motors to be equally distant from throttle value for balanced motor output during hard yaw
  byte motorMaxCheck = OFF;
  byte motorMinCheck = OFF;

  // Check if everything within motor limits
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    motorMaxCheck = motorMaxCheck | (motorCommand[motor] >= MAXCOMMAND);
    motorMinCheck = motorMinCheck | (motorCommand[motor] <= minArmedThrottle);
  }

  // If everything within limits, turn flags off
  if (!motorMaxCheck) {
    if (maxLimit) {
      for (byte motor = 0; motor < LASTMOTOR; motor++)
        motorMinLimitCommand[motor] = minArmedThrottle;
      maxLimit = OFF;
    }
  }
  if (!motorMinCheck) {
    if (minLimit) {
      for (byte motor = 0; motor < LASTMOTOR; motor++)
        motorMaxLimitCommand[motor] = MAXCOMMAND;
      minLimit = OFF;
    }
  }

  // If any limits reached, freeze current min values and turn limit flag on
  // In future iterations, if limit still exceeded still use first min values
  for (byte motor = 0; motor < LASTMOTOR; motor++) {
    if ((motorCommand[motor] >= MAXCOMMAND) && maxLimit == OFF) {
      for (byte motorLimit = 0; motorLimit < LASTMOTOR; motorLimit++)
        motorMinLimitCommand[motorLimit] = motorCommand[motorLimit];
      maxLimit = ON;
    }
    if ((motorCommand[motor] <= minArmedThrottle) && minLimit == OFF) {
      for (byte motorLimit = 0; motorLimit < LASTMOTOR; motorLimit++)
        motorMaxLimitCommand[motorLimit] = motorCommand[motorLimit];
      minLimit = ON;
    }
    motorCommand[motor] = constrain(motorCommand[motor], motorMinLimitCommand[motor], motorMaxLimitCommand[motor]);
  }
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

