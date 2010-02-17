/*
  AeroQuad v1.5 - Novmeber 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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

#ifndef MOTORS_H
#define MOTORS_H

#define FRONTMOTORPIN 3
#define REARMOTORPIN 9
#define RIGHTMOTORPIN 10
#define LEFTMOTORPIN 11
#define LASTMOTORPIN 12
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define LASTMOTOR 4

// Scale motor commands to analogWrite		
// m = (250-126)/(2000-1000) = 0.124		
// b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
float mMotorCommand = 0.124;		
float bMotorCommand = 2;

int motorCommand[4] = {1000,1000,1000,1000};
int motorAxisCommand[3] = {0,0,0};
int motor = 0;
// If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V 
// Scale gyro output (-465 to +465) to motor commands (1000 to 2000) 
// use y = mx + b 
float mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465)) 
float bMotorRate = 1500;   // b = y1 - m * x1

int minCommand[4] = {MINCOMMAND, MINCOMMAND, MINCOMMAND,MINCOMMAND};
int maxCommand[4] = {MAXCOMMAND, MAXCOMMAND, MAXCOMMAND,MAXCOMMAND};
int delta = 0;

void configureMotors();
void commandMotors();
void commandAllMotors(int motorCommand);
void pulseMotors(byte quantity);

#endif
