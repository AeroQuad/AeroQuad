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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_

#include "FlightControlVariable.h"

/*  

                 CCW
            
       CW  0....Front....0  CW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CCW
      
                 CW
*/     


#define FRONT       MOTOR1
#define FRONT_RIGHT MOTOR2
#define REAR_RIGHT  MOTOR3
#define REAR        MOTOR4
#define REAR_LEFT   MOTOR5
#define FRONT_LEFT  MOTOR6
#define LASTMOTOR   MOTOR6+1

int motorMaxCommand[6] = {0,0,0,0,0,0};
int motorMinCommand[6] = {0,0,0,0,0,0};
int motorConfiguratorCommand[6] = {0,0,0,0,0,0};

void applyMotorCommand() {
  const int throttleCorrection = abs(motorAxisCommandYaw*3/6);
  motorCommand[FRONT_LEFT]  = (throttle - throttleCorrection) + motorAxisCommandRoll/2 - motorAxisCommandPitch/2 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = (throttle - throttleCorrection) - motorAxisCommandRoll/2 + motorAxisCommandPitch/2 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = (throttle - throttleCorrection) - motorAxisCommandRoll/2 - motorAxisCommandPitch/2 - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = (throttle - throttleCorrection) + motorAxisCommandRoll/2 + motorAxisCommandPitch/2 + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT]       = (throttle - throttleCorrection)                          - motorAxisCommandPitch   + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        = (throttle - throttleCorrection)                          + motorAxisCommandPitch   - (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif  // #define _AQ_PROCESS_FLIGHT_CONTROL_HEX_PLUS_MODE_H_

