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


#ifndef _AEROQUAD_MOTORS_H_
#define _AEROQUAD_MOTORS_H_

#include "Arduino.h"

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3
#define MOTOR5 4
#define MOTOR6 5
#define MOTOR7 6
#define MOTOR8 7
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

enum NB_Motors{
  FOUR_Motors = 4,
  SIX_Motors = 6,
  EIGHT_Motors = 8
};

NB_Motors numberOfMotors = FOUR_Motors;
int motorCommand[8] = {0,0,0,0,0,0,0,0};  // LASTMOTOR not know here, so, default at 8 @todo : Kenny, find a better way
  
void initializeMotors(NB_Motors numbers = FOUR_Motors);
void writeMotors();
void commandAllMotors(int command);

void pulseMotors(byte nbPulse) {
  for (byte i = 0; i < nbPulse; i++) {
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}


#endif