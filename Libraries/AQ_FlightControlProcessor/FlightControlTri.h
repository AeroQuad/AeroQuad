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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_TRI_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_TRI_MODE_H_


/*
       CW                  CCW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
                  CW           
*/

#include "FlightControlVariable.h"

#define SERVO       MOTOR1
#define FRONT_LEFT  MOTOR2
#define FRONT_RIGHT MOTOR3
#define REAR        MOTOR4
#define LASTMOTOR   (MOTOR4+1)

#define TRI_YAW_CONSTRAINT_MIN 1100
#define TRI_YAW_CONSTRAINT_MAX 1900
#define TRI_YAW_MIDDLE 1500

#define MAX_RECEIVER_OFFSET 50

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  motorCommand[FRONT_LEFT]    = throttle + motorAxisCommandRoll - motorAxisCommandPitch*2/3;
  motorCommand[FRONT_RIGHT]   = throttle - motorAxisCommandRoll - motorAxisCommandPitch*2/3;
  motorCommand[REAR]          = throttle + motorAxisCommandPitch*4/3;
  const float yawMotorCommand = constrain(motorAxisCommandYaw,-MAX_RECEIVER_OFFSET-abs(receiverCommand[ZAXIS]),+MAX_RECEIVER_OFFSET+abs(receiverCommand[ZAXIS]));
  motorCommand[SERVO]         = constrain(TRI_YAW_MIDDLE + YAW_DIRECTION * yawMotorCommand, TRI_YAW_CONSTRAINT_MIN, TRI_YAW_CONSTRAINT_MAX);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
