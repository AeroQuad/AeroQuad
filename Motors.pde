/*
  AeroQuad v1.4 - September 2009
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

#include "Motors.h"

void configureMotors() {
  #ifdef ServoTimerTwo  
    frontMotor.attach(FRONTMOTORPIN);
    rearMotor.attach(REARMOTORPIN);
    rightMotor.attach(RIGHTMOTORPIN);
    leftMotor.attach(LEFTMOTORPIN);
  #endif
  #ifdef AnalogWrite
    for (motor = FRONTMOTORPIN; motor < LASTMOTORPIN; motor++) {
      analogWrite(FRONTMOTORPIN, 124);
      analogWrite(REARMOTORPIN, 124);
      analogWrite(RIGHTMOTORPIN, 124);
      analogWrite(LEFTMOTORPIN, 124);
    }
  #endif
}

void commandMotors() {
  #ifdef ServoTimerTwo
    frontMotor.write(motorCommand[FRONT]);
    rearMotor.write(motorCommand[REAR]);
    rightMotor.write(motorCommand[RIGHT]);
    leftMotor.write(motorCommand[LEFT]);
  #endif
  #ifdef AnalogWrite
    analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);
    analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);
    analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);
    analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);
  #endif
}

// Sends commands to all motors
void commandAllMotors(int motorCommand) {
  #ifdef ServoTimerTwo
    frontMotor.write(motorCommand);
    rearMotor.write(motorCommand);
    rightMotor.write(motorCommand);
    leftMotor.write(motorCommand);
  #endif
  #ifdef AnalogWrite
    analogWrite(FRONTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);
    analogWrite(REARMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);
    analogWrite(RIGHTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);
    analogWrite(LEFTMOTORPIN, (motorCommand * mMotorCommand) + bMotorCommand);
  #endif
  }

void pulseMotors(byte quantity) {
  for (byte i = 0; i < quantity; i++) {      
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}
