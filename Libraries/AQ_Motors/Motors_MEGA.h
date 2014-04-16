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


#ifndef _AEROQUAD_MOTORS_MEGA_H_
#define _AEROQUAD_MOTORS_MEGA_H_

#include "Arduino.h"

#include "Motors.h"

#define MOTORPIN0    2
#define MOTORPIN1    3
#define MOTORPIN2    5
#define MOTORPIN3    6
#define MOTORPIN4    7
#define MOTORPIN5    8
#define MOTORPIN6    11
#define MOTORPIN7    12
  
#define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT); //PIN 2 //also right servo for BI COPTER
#define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
#define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);

volatile uint8_t atomicServo = 125;
  
void initializeServo() {
  DIGITAL_SERVO_TRI_PINMODE
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}
  

// 6 motors ISR
ISR(TIMER0_COMPA_vect) {

  static uint8_t state = 0;
  if (flightConfigType == TRI) {
    static uint8_t count;
    if (state == 0) {
      //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
      DIGITAL_SERVO_TRI_HIGH
      OCR0A+= 250; // 1000 us
      state++ ;
    } 
	else if (state == 1) {
      OCR0A+= atomicServo; // 1000 + [0-1020] us
      state++;
    } 
	else if (state == 2) {
      DIGITAL_SERVO_TRI_LOW
      OCR0A+= 250; // 1000 us
      state++;
    } 
	else if (state == 3) {
      state++;
    } 
	else if (state == 4) {
      state++;
      OCR0A+= 250; // 1000 us
    } 
	else if (state == 5) {
      state++;
    } 
	else if (state == 6) {
      state++;
      OCR0A+= 250; // 1000 us
    } 
	else if (state == 7) {
      state++;
    } 
	else if (state == 8) {
      count = 10; // 12 x 1000 us
      state++;
      OCR0A+= 250; // 1000 us
    } 
	else if (state == 9) {
      if (count > 0) {
	    count--;
	  }
      else {
	    state = 0;
	  }
      OCR0A+= 250;
    }
  }
}


void initializeMotors(byte numbers) {

  numberOfMotors = numbers;
  if (flightConfigType == TRI) {
    initializeServo();
  }
  else {
    pinMode(MOTORPIN0, OUTPUT);
  }
  pinMode(MOTORPIN1, OUTPUT);
  pinMode(MOTORPIN2, OUTPUT);
  pinMode(MOTORPIN3, OUTPUT);
  if (numberOfMotors == 6 || numberOfMotors == 8) {
    pinMode(MOTORPIN4, OUTPUT);
    pinMode(MOTORPIN5, OUTPUT);
  }
  if (numberOfMotors == 8) {
    pinMode(MOTORPIN6, OUTPUT);
    pinMode(MOTORPIN7, OUTPUT);
  }
    
  commandAllMotors(1000);
}

void writeMotors() {

  if (flightConfigType == TRI) {
    atomicServo = (motorCommand[MOTOR1]-1000)/4;
  }
  else {
    analogWrite(MOTORPIN0, motorCommand[MOTOR1] / 8);
  }
  analogWrite(MOTORPIN1, motorCommand[MOTOR2] / 8);
  analogWrite(MOTORPIN2, motorCommand[MOTOR3] / 8);
  analogWrite(MOTORPIN3, motorCommand[MOTOR4] / 8); 
  if (numberOfMotors == 6 || numberOfMotors == 8) {
    analogWrite(MOTORPIN4, motorCommand[MOTOR5] / 8);
    analogWrite(MOTORPIN5, motorCommand[MOTOR6] / 8);
  }
  if (numberOfMotors == 8) {
    analogWrite(MOTORPIN6, motorCommand[MOTOR7] / 8);
    analogWrite(MOTORPIN7, motorCommand[MOTOR8] / 8);
  }
}

void commandAllMotors(int command) {

  if (flightConfigType == TRI) {
    atomicServo = (command-1000)/4;
  }
  else {
    analogWrite(MOTORPIN0, command / 8);
  }
  analogWrite(MOTORPIN1, command / 8);
  analogWrite(MOTORPIN2, command / 8);
  analogWrite(MOTORPIN3, command / 8);
  if (numberOfMotors == 6 || numberOfMotors == 8) {
    analogWrite(MOTORPIN4, command / 8);
    analogWrite(MOTORPIN5, command / 8);
  }
  if (numberOfMotors == 8) {
    analogWrite(MOTORPIN6, command / 8);
    analogWrite(MOTORPIN7, command / 8);
  }
}  


#endif