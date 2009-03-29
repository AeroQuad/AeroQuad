/*
  MikroQuad v3.0 - March 2009
  www.MikroQuad.com
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

void configureMotors() {
  frontMotor.attach(FRONTMOTORPIN);
  rearMotor.attach(REARMOTORPIN);
  rightMotor.attach(RIGHTMOTORPIN);
  leftMotor.attach(LEFTMOTORPIN);
}

void commandMotors() {
  frontMotor.write(motorCommand[FRONT]);
  rearMotor.write(motorCommand[REAR]);
  rightMotor.write(motorCommand[RIGHT]);
  leftMotor.write(motorCommand[LEFT]);
}

// Sends commands to all motors
void commandAllMotors(int motorCommand) {
  frontMotor.write(motorCommand);
  rearMotor.write(motorCommand);
  rightMotor.write(motorCommand);
  leftMotor.write(motorCommand);
}

void pulseMotors(byte quantity) {
  for (byte i=0; i<quantity; i++) {      
    commandAllMotors(MINCOMMAND + 50);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}

/*
void armESC(unsigned long msWaitTime) {
  unsigned long startTime;
  startTime = millis();
  while ((millis() - startTime) < msWaitTime) {
    digitalWrite(FRONTMOTORPIN, HIGH);
    digitalWrite(REARMOTORPIN, HIGH);
    digitalWrite(RIGHTMOTORPIN, HIGH);
    digitalWrite(LEFTMOTORPIN, HIGH);
    delay(1);
    digitalWrite(FRONTMOTORPIN, LOW);
    digitalWrite(REARMOTORPIN, LOW);
    digitalWrite(RIGHTMOTORPIN, LOW);
    digitalWrite(LEFTMOTORPIN, LOW);
    delay(20);
  }
}
*/
