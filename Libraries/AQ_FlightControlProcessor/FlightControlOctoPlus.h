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

//#define FRONT       MOTOR1
//#define FRONT_RIGHT MOTOR2
//#define RIGHT       MOTOR3
//#define REAR_RIGHT  MOTOR4
//#define REAR        MOTOR5
//#define REAR_LEFT   MOTOR6
//#define LEFT        MOTOR7
//#define FRONT_LEFT  MOTOR8
//#define LASTMOTOR   (MOTOR8+1)

void applyMotorCommandOctoPlus() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  motorCommand[MOTOR1] = throttle - motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR2] = throttle - motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR3] = throttle                              - motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR4] = throttle + motorAxisCommandPitch*7/10 - motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR5] = throttle + motorAxisCommandPitch                                  - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR6] = throttle + motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR7] = throttle                              + motorAxisCommandRoll      - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[MOTOR8] = throttle - motorAxisCommandPitch*7/10 + motorAxisCommandRoll*7/10 + (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_PLUS_MODE_H_

