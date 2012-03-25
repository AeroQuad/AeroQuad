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


#ifndef _AEROQUAD_MOTORS_PWM_H_
#define _AEROQUAD_MOTORS_PWM_H_

#include "Arduino.h"
#include "Motors.h"

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MOTORPIN0    2
  #define MOTORPIN1    3
  #define MOTORPIN2    5
  #define MOTORPIN3    6
  #define MOTORPIN4    7
  #define MOTORPIN5    8
  #define MOTORPIN6    9
  #define MOTORPIN7    10
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(2,OUTPUT); //PIN 2 //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTE |= 1<<4;
  #define DIGITAL_SERVO_TRI_LOW      PORTE &= ~(1<<4);
#else
  #define MOTORPIN0    3
  #define MOTORPIN1    9
  #define MOTORPIN2   10
  #define MOTORPIN3   11
  #define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT); //also right servo for BI COPTER
  #define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
  #define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);
#endif

volatile uint8_t atomicServo = 125;


void initializeServo() {
  DIGITAL_SERVO_TRI_PINMODE
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}

// ****servo yaw with a 50Hz refresh rate****
// prescaler is set by default to 64 on Timer0
// Duemilanove : 16MHz / 64 => 4 us
// 256 steps = 1 counter cycle = 1024 us
// algorithm strategy:
// pulse high servo 0 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 0
// pulse high servo 1 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 1
// pulse high servo 2 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 2
// pulse high servo 3 -> do nothing for 1000 us -> do nothing for [0 to 1000] us -> pulse down servo 3
// do nothing for 14 x 1000 us
ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  static uint8_t count;
  if (state == 0) {
    //http://billgrundmann.wordpress.com/2009/03/03/to-use-or-not-use-writedigital/
    DIGITAL_SERVO_TRI_HIGH
    OCR0A+= 250; // 1000 us
    state++ ;
  } else if (state == 1) {
    OCR0A+= atomicServo; // 1000 + [0-1020] us
    state++;
  } else if (state == 2) {
    DIGITAL_SERVO_TRI_LOW
    OCR0A+= 250; // 1000 us
    state++;
  } else if (state == 3) {
    state++;
  } else if (state == 4) {
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 5) {
    state++;
  } else if (state == 6) {
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 7) {
    state++;
  } else if (state == 8) {
    count = 10; // 12 x 1000 us
    state++;
    OCR0A+= 250; // 1000 us
  } else if (state == 9) {
    if (count > 0) count--;
    else state = 0;
    OCR0A+= 250;
  }
}


void initializeMotors(NB_Motors numbers) {
  
  numberOfMotors = numbers;

  pinMode(MOTORPIN0, OUTPUT);
  pinMode(MOTORPIN1, OUTPUT);
  pinMode(MOTORPIN2, OUTPUT);
  pinMode(MOTORPIN3, OUTPUT);
  
  initializeServo();
  
  commandAllMotors(1000);
}

void writeMotors() {

  atomicServo = (motorCommand[MOTOR1]-1000)/4;
  analogWrite(MOTORPIN1, motorCommand[MOTOR2] / 8);
  analogWrite(MOTORPIN2, motorCommand[MOTOR3] / 8);
  analogWrite(MOTORPIN3, motorCommand[MOTOR4] / 8); 
}

void commandAllMotors(int command) {

  atomicServo = (command-1000)/4;
  analogWrite(MOTORPIN1, command / 8);
  analogWrite(MOTORPIN2, command / 8);
  analogWrite(MOTORPIN3, command / 8);
}  


#endif