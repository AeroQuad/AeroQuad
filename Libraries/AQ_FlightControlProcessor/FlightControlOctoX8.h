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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_


/*  
             UPPER/LOWER


       CW/CCW           CCW/CW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
       CCW/CW           CW/CCW
*/     

#include "FlightControlVariable.h"

#define FRONT_LEFT    MOTOR1
#define FRONT_RIGHT   MOTOR2
#define REAR_RIGHT    MOTOR3
#define REAR_LEFT     MOTOR4
#define FRONT_LEFT_2  MOTOR5
#define FRONT_RIGHT_2 MOTOR6
#define REAR_RIGHT_2  MOTOR7
#define REAR_LEFT_2   MOTOR8
#define LASTMOTOR     MOTOR8+1

int motorMaxCommand[8] = {0,0,0,0,0,0,0,0};
int motorMinCommand[8] = {0,0,0,0,0,0,0,0};
int motorConfiguratorCommand[8] = {0,0,0,0,0,0,0,0};

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  const int throttleCorrection = abs(motorAxisCommandYaw*4/8);
  motorCommand[FRONT_LEFT] =    (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] =   (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT] =     (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT] =    (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_LEFT_2] =  (throttle-throttleCorrection) - motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT_2] = (throttle-throttleCorrection) - motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT_2] =   (throttle-throttleCorrection) + motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT_2] =  (throttle-throttleCorrection) + motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_OCTO_X8_MODE_H_

