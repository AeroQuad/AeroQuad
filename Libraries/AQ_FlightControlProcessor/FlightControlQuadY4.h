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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_

/*
             UPPER/LOWER


       CW                  CCW
            
           0....Front....0  
           ......***......    
           ......***......
           ......***......    
           0....Back.....0  
      
                CW/CCW           
*/


#include "FlightControlVariable.h"

#define LEFT        MOTOR1
#define RIGHT       MOTOR2
#define REAR        MOTOR3
#define REAR_UNDER  MOTOR4
#define LASTMOTOR   (MOTOR4+1)

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  // Front = Front/Right, Back = Left/Rear, Left = Front/Left, Right = Right/Rear 
  motorCommand[LEFT]        = throttle - motorAxisCommandPitch + motorAxisCommandRoll;
  motorCommand[RIGHT]       = throttle - motorAxisCommandPitch - motorAxisCommandRoll;
  motorCommand[REAR_UNDER]  = throttle + motorAxisCommandPitch + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR]        = throttle + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_Y4_MODE_H_
