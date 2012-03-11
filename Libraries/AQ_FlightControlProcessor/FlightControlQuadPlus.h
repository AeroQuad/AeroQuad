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

#ifndef _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_

/*  
                 CW
            
           0....Front....0  
           ......***......    
      CCW  ......***......  CCW
           ......***......    
           0....Back.....0  
      
                 CW
*/     

#include "FlightControlVariable.h"

#ifdef OLD_MOTOR_NUMBERING  
  #define FRONT MOTOR1
  #define REAR  MOTOR2
  #define RIGHT MOTOR3
  #define LEFT  MOTOR4
#else
  #define FRONT MOTOR1
  #define RIGHT MOTOR2
  #define REAR  MOTOR3
  #define LEFT  MOTOR4
#endif
#define LASTMOTOR MOTOR4+1

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  const int throttleCorrection = abs(motorAxisCommandYaw*2/4);
  motorCommand[FRONT] = (throttle - throttleCorrection) - motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR] =  (throttle - throttleCorrection) + motorAxisCommandPitch - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[RIGHT] = (throttle - throttleCorrection) - motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT] =  (throttle - throttleCorrection) + motorAxisCommandRoll  + (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_PLUS_MODE_H_
