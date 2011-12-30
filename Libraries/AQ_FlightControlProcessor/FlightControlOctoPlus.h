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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_


/*  
             UPPER/LOWER


                 CW
       CCW                CCW
           0....Front....0  
           ......***......    
     CW    ......***......   CW
           ......***......    
           0....Back.....0  
       CCW                CCW
                 CW
*/     

#include "FlightControlVariable.h"

#define FRONT       MOTOR1
#define FRONT_RIGHT MOTOR2
#define RIGHT       MOTOR3
#define REAR_RIGHT  MOTOR5
#define REAR        MOTOR4
#define REAR_LEFT   MOTOR6
#define LEFT        MOTOR7
#define FRONT_LEFT  MOTOR8
#define LASTMOTOR   MOTOR8+1

int motorMaxCommand[8] = {0,0,0,0,0,0,0,0};
int motorMinCommand[8] = {0,0,0,0,0,0,0,0};
int motorConfiguratorCommand[8] = {0,0,0,0,0,0,0,0};

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection  = abs(motorAxisCommandYaw*4/8);
  motorCommand[FRONT]       = (throttle-throttleCorrection) - motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle-throttleCorrection) - motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT]       = (throttle-throttleCorrection)                              - motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = (throttle-throttleCorrection) + motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        = (throttle-throttleCorrection) + motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = (throttle-throttleCorrection) + motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT]        = (throttle-throttleCorrection)                              + motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_LEFT]  = (throttle-throttleCorrection) - motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_

