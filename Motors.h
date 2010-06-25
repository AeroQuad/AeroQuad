/*
  AeroQuad v1.8 - June 2010
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

#ifdef APM
#define FRONTMOTORPIN 8
#define REARMOTORPIN 7
#define RIGHTMOTORPIN 3
#define LEFTMOTORPIN 2
#endif

#if defined(AeroQuad_v1) || defined (AeroQuad_Wii)
#define FRONTMOTORPIN 3
#define REARMOTORPIN 9
#define RIGHTMOTORPIN 10
#define LEFTMOTORPIN 11
#define LASTMOTORPIN 12
#endif

#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define LASTMOTOR 4
#define MINCOMMAND 1000
#define MAXCOMMAND 2000

// Motors
// Assume maximum number of motors is 8, leave array indexes unused if lower number
//int motorCommand[4] = {1000,1000,1000,1000};
//int minCommand[4] = {MINCOMMAND, MINCOMMAND, MINCOMMAND,MINCOMMAND};
//int maxCommand[4] = {MAXCOMMAND, MAXCOMMAND, MAXCOMMAND,MAXCOMMAND};
int motorAxisCommand[3] = {0,0,0};
int motorAxisCommandRoll[8] = {0,0,0,0,0,0,0,0};
int motorAxisCommandPitch[8] = {0,0,0,0,0,0,0,0};
int motorAxisCommandYaw[8] = {0,0,0,0,0,0,0,0};
int motorMixerSettingRoll[8] = {0,0,0,0,0,0,0,0};
int motorMixerSettingPitch[8] = {0,0,0,0,0,0,0,0};
int motorMixerSettingYaw[8] = {0,0,0,0,0,0,0,0};
int motorCommand[8] = {1000,1000,1000,1000,1000,1000,1000,1000};
int minCommand[8] = {MINCOMMAND, MINCOMMAND, MINCOMMAND,MINCOMMAND,MINCOMMAND, MINCOMMAND, MINCOMMAND,MINCOMMAND};
int maxCommand[8] = {MAXCOMMAND, MAXCOMMAND, MAXCOMMAND,MAXCOMMAND,MAXCOMMAND, MAXCOMMAND, MAXCOMMAND,MAXCOMMAND};
int motor = 0;
int delta = 0;

// Ground station control
//int remoteCommand[4] = {1000,1000,1000,1000};
int remoteCommand[8] = {1000,1000,1000,1000,1000,1000,1000,1000};

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
    // Only supports commands from 0-255 => 0 - 100% duty cycle
    // Usable pulsewith from approximately 1000-2000 us = 126 - 250	
    // m = (250-126)/(2000-1000) = 0.124		
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
    mMotorCommand = 0.124;		
    bMotorCommand = 2.0;

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

// Need to set this up as a #define because Duemilanove does not have Mega/APM register references
#ifdef APM
class Motors_APM {
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
  Motors_APM() {
    mMotorCommand = 1.0;		
    bMotorCommand = 0.0;

    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      //motorCommand[motor] = 1000;
      remoteCommand[motor] = 1000;
    }
    //for (axis = ROLL; axis < LASTAXIS; axis++)
    //  motorAxisCommand[axis] = 0;
    motor = 0;
  }

  void initialize(void) {
    // Init PWM Timer 3
    pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
    pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
    //pinMode(4,OUTPUT); //     (PE3/OC3A)
    TCCR3A =((1<<WGM31)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
    //OCR3A = 3000; //PE3, NONE
    OCR3B = 3000; //PE4, OUT7
    OCR3C = 3000; //PE5, OUT6
    //ICR3 = 40000; //50hz freq (standard servos)
    ICR3 = 6600; //50hz freq (standard servos)

    // Init PWM Timer 5
    pinMode(44,OUTPUT);  //OUT1 (PL5/OC5C)
    pinMode(45,OUTPUT);  //OUT0 (PL4/OC5B)
    //pinMode(46,OUTPUT);  //     (PL3/OC5A)
    
    TCCR5A =((1<<WGM51)|(1<<COM5B1)|(1<<COM5C1)); 
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    //OCR5A = 3000; //PL3, 
    OCR5B = 3000; //PL4, OUT0
    OCR5C = 3000; //PL5, OUT1
    ICR5 = 6600; //300hz freq
  }

  void write (int *motorCommand) {
    OCR5B = motorCommand[FRONT] * 2;
    OCR5C = motorCommand[REAR] * 2;
    OCR3C = motorCommand[RIGHT] * 2;
    OCR3B = motorCommand[LEFT] * 2;
  }
  
  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void commandAllMotors(int motorCommand) {   // Sends commands to all motors
    OCR5B=motorCommand * 2;
    OCR5C=motorCommand * 2;
    OCR1B=motorCommand * 2;
    OCR1C=motorCommand * 2;
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
