/*
  AeroQuad v1.6 - February 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

// Motors
int motorCommand[4] = {1000,1000,1000,1000};
int motorAxisCommand[3] = {0,0,0};
int motor = 0;
int minCommand[4] = {MINCOMMAND, MINCOMMAND, MINCOMMAND,MINCOMMAND};
int maxCommand[4] = {MAXCOMMAND, MAXCOMMAND, MAXCOMMAND,MAXCOMMAND};
int delta = 0;
// If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V 
// Scale gyro output (-465 to +465) to motor commands (1000 to 2000) 
// use y = mx + b 
float mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465)) 
float bMotorRate = 1500;   // b = y1 - m * x1
// Ground station control
int remoteCommand[4] = {1000,1000,1000,1000};

class Motors_PWM {
private:
  int motor;
  float mMotorCommand;
  float bMotorCommand;
  int minCommand;
  byte axis;
  byte pin;
  
  // Ground station control (experimental)
  int remoteCommand[4];

public:
  Motors_PWM() {
    // Scale motor commands to analogWrite		
    // m = (250-126)/(2000-1000) = 0.124		
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
    mMotorCommand = 0.124;		
    bMotorCommand = 2;

    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      //motorCommand[motor] = 1000;
      remoteCommand[motor] = 1000;
    }
    //for (axis = ROLL; axis < LASTAXIS; axis++)
    //  motorAxisCommand[axis] = 0;
    motor = 0;
  }

  void initialize(void) {
    pinMode(FRONTMOTORPIN, OUTPUT);
    analogWrite(FRONTMOTORPIN, 124);		
    pinMode(REARMOTORPIN, OUTPUT);
    analogWrite(REARMOTORPIN, 124);		
    pinMode(RIGHTMOTORPIN, OUTPUT);
    analogWrite(RIGHTMOTORPIN, 124);		
    pinMode(LEFTMOTORPIN, OUTPUT);
    analogWrite(LEFTMOTORPIN, 124);
  }

  void write (int *motorCommand) {
    analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);
    analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);
    analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);
    analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);
  }
  
  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void commandAllMotors(int motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, (motorCommand * mMotorCommand + bMotorCommand));
    analogWrite(REARMOTORPIN, (motorCommand * mMotorCommand + bMotorCommand));		
    analogWrite(RIGHTMOTORPIN, (motorCommand * mMotorCommand + bMotorCommand));		
    analogWrite(LEFTMOTORPIN, (motorCommand * mMotorCommand + bMotorCommand));
  }

  void pulseMotors(byte quantity) {
    for (byte i = 0; i < quantity; i++) {      
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }
  
  void setRemoteMotorCommand(byte motor, int value) {
    remoteCommand[motor] = value;
 }
  
  int getRemoteMotorCommand(byte motor) {
    return remoteCommand[motor];
  }
  
  int getMotorCommand(byte motor) {
    return motorCommand[motor];
  }
  
  float getMotorSlope(void) {
    return mMotorCommand;
  }
  
  float getMotorOffset(void) {
    return bMotorCommand;
  }
};

#endif
