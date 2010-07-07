/*
  AeroQuad v2.0 - July 2010
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

class Motors {
public:
  // Assume maximum number of motors is 8, leave array indexes unused if lower number
  int motorAxisCommand[3];
  int motorAxisCommandRoll[8];
  int motorAxisCommandPitch[8];
  int motorAxisCommandYaw[8];
  int motorMixerSettingRoll[8];
  int motorMixerSettingPitch[8];
  int motorMixerSettingYaw[8];
  int motorCommand[8];
  int minCommand[8];
  int maxCommand[8];
  int motor;
  int delta;
  byte axis;
  // Ground station control
  int remoteCommand[8];
  float mMotorCommand;
  float bMotorCommand;


  Motors(void){
    motorAxisCommand[ROLL] = 0;
    motorAxisCommand[PITCH] = 0;
    motorAxisCommand[YAW] = 0;
    for (motor = 0; motor < 8; motor++) {
      motorAxisCommandRoll[motor] = 0;
      motorAxisCommandPitch[motor] = 0;
      motorAxisCommandYaw[motor] = 0;
      motorMixerSettingRoll[motor] = 0;
      motorMixerSettingPitch[motor] = 0;
      motorMixerSettingYaw[motor] = 0;
      motorCommand[motor] = 1000;
      minCommand[motor] = MINCOMMAND;
      maxCommand[motor] = MAXCOMMAND;
      remoteCommand[motor] = 1000;
    }
    motor = 0;
    delta = 0;  
  };
  
  // The following function calls must be defined in any new subclasses
  virtual void initialize(void);
  virtual void write (void);
  virtual void commandAllMotors(int motorCommand);
  
  //Any number of optional methods can be configured as needed by the SubSystem to expose functionality externally
  void pulseMotors(byte quantity) {
    for (byte i = 0; i < quantity; i++) {      
      commandAllMotors(MINCOMMAND + 100);
      delay(250);
      commandAllMotors(MINCOMMAND);
      delay(250);
    }
  }
  
  void setRemoteCommand(byte motor, int value) {
    remoteCommand[motor] = value;
  }
  
  const int getRemoteCommand(byte motor) {
    return remoteCommand[motor];
  }
  
  const float getMotorSlope(void) {
    return mMotorCommand;
  }
  
  const float getMotorOffset(void) {
    return bMotorCommand;
  }
    
  void setMinCommand(byte motor, int value) {
    minCommand[motor] = value;
  }
  
  const int getMinCommand(byte motor) {
    return minCommand[motor];
  }
  
  void setMaxCommand(byte motor, int value) {
    maxCommand[motor] = value;
  }
  
  const int getMaxCommand(byte motor) {
    return maxCommand[motor];
  }
  
  void setMotorAxisCommand(byte motor, int value) {
    motorAxisCommand[motor] = value;
  }
  
  const int getMotorAxisCommand(byte motor) {
    return motorAxisCommand[motor];
  }
  
  void setMotorCommand(byte motor, int value) {
    motorCommand[motor] = value;
  }
  
  const int getMotorCommand(byte motor) {
    return motorCommand[motor];
  }
};

/******************************************************/
/********************* PWM Motors *********************/
/******************************************************/
class Motors_PWM : public Motors {
private:
  int minCommand;
  byte pin;
  
  #define FRONTMOTORPIN 3
  #define REARMOTORPIN 9
  #define RIGHTMOTORPIN 10
  #define LEFTMOTORPIN 11
  #define LASTMOTORPIN 12

public:
  Motors_PWM() : Motors(){
    // Scale motor commands to analogWrite
    // Only supports commands from 0-255 => 0 - 100% duty cycle
    // Usable pulsewith from approximately 1000-2000 us = 126 - 250	
    // m = (250-126)/(2000-1000) = 0.124		
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2		
    mMotorCommand = 0.124;		
    bMotorCommand = 2.0;
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

  void write (void) {
    analogWrite(FRONTMOTORPIN, (motorCommand[FRONT] * mMotorCommand) + bMotorCommand);
    analogWrite(REARMOTORPIN, (motorCommand[REAR] * mMotorCommand) + bMotorCommand);
    analogWrite(RIGHTMOTORPIN, (motorCommand[RIGHT] * mMotorCommand) + bMotorCommand);
    analogWrite(LEFTMOTORPIN, (motorCommand[LEFT] * mMotorCommand) + bMotorCommand);
  }
  
  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    analogWrite(FRONTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
    analogWrite(REARMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(RIGHTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);		
    analogWrite(LEFTMOTORPIN, (_motorCommand * mMotorCommand) + bMotorCommand);
  }
};

/******************************************************/
/********************* APM Motors *********************/
/******************************************************/
#ifdef APM
class Motors_APM : public Motors {
public:
  Motors_APM() : Motors() {}

  void initialize(void) {
    // Init PWM Timer 1
    //pinMode(11,OUTPUT); //     (PB5/OC1A)
    pinMode(12,OUTPUT); //OUT2 (PB6/OC1B)
    pinMode(13,OUTPUT); //OUT3 (PB7/OC1C)
  
    //Remember the registers not declared here remains zero by default... 
    TCCR1A =((1<<WGM11)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all... 
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
    //OCR1A = 3000; //PB5, none
    OCR1B = 3000; //PB6, OUT2
    OCR1C = 3000; //PB7  OUT3
    ICR1 = 6600; //300hz freq...
  
    // Init PWM Timer 3
    pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
    pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
    //pinMode(4,OUTPUT); //     (PE3/OC3A)
    TCCR3A =((1<<WGM31)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31); 
    //OCR3A = 3000; //PE3, NONE
    OCR3B = 3000; //PE4, OUT7
    OCR3C = 3000; //PE5, OUT6
    ICR3 = 40000; //50hz freq (standard servos)
  
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
  
    // Init PPM input and PWM Timer 4
    pinMode(49, INPUT);  // ICP4 pin (PL0) (PPM input)
    pinMode(7,OUTPUT);   //OUT5 (PH4/OC4B)
    pinMode(8,OUTPUT);   //OUT4 (PH5/OC4C)
        
    TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));  
    //Prescaler set to 8, that give us a resolution of 0.5us
    // Input Capture rising edge
    TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4));
    
    OCR4A = 40000; ///50hz freq. (standard servos)
    OCR4B = 3000; //PH4, OUT5
    OCR4C = 3000; //PH5, OUT4
   
    //TCCR4B |=(1<<ICES4); //Changing edge detector (rising edge). 
    //TCCR4B &=(~(1<<ICES4)); //Changing edge detector. (falling edge)
    TIMSK4 |= (1<<ICIE4); // Enable Input Capture interrupt. Timer interrupt mask
  }

  void write (void) {
    OCR1B = motorCommand[FRONT] * 2;
    OCR1C = motorCommand[REAR] * 2;
    OCR5B = motorCommand[RIGHT] * 2;
    OCR5C = motorCommand[LEFT] * 2;
  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
    OCR1B = _motorCommand * 2;
    OCR1C = _motorCommand * 2;
    OCR5B = _motorCommand * 2;
    OCR5C = _motorCommand * 2;
  }
};
#endif
