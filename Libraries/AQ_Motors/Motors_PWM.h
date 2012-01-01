/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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
  #define MOTORPIN6    11
  #define MOTORPIN7    12
#else
  #define MOTORPIN0    3
  #define MOTORPIN1    9
  #define MOTORPIN2   10
  #define MOTORPIN3   11
  #define MOTORPIN4    5   
  #define MOTORPIN5    6  
  
volatile uint8_t atomicPWM_PIN5_lowState = 0;
volatile uint8_t atomicPWM_PIN5_highState = 0;
volatile uint8_t atomicPWM_PIN6_lowState = 0;
volatile uint8_t atomicPWM_PIN6_highState = 0;
  
void initializeSoftPWM() {
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<5; //digital PIN 5 high
    OCR0A+= atomicPWM_PIN5_highState; //250 x 4 microsecons = 1ms
    state = 1;
  } else if (state == 1) {
    OCR0A+= atomicPWM_PIN5_highState;
    state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<5); //digital PIN 5 low
    OCR0A+= atomicPWM_PIN5_lowState;
    state = 0;
  }
}

ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (state == 0) {
    PORTD |= 1<<6;OCR0B+= atomicPWM_PIN6_highState;state = 1;
  } else if (state == 1) {
    OCR0B+= atomicPWM_PIN6_highState;state = 2;
  } else if (state == 2) {
    PORTD &= ~(1<<6);OCR0B+= atomicPWM_PIN6_lowState;state = 0;
  }
}

#endif


void initializeMotors(NB_Motors numbers) {
  numberOfMotors = numbers;
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	  
  #else
    pinMode(MOTORPIN0, OUTPUT);
    pinMode(MOTORPIN1, OUTPUT);
    pinMode(MOTORPIN2, OUTPUT);
    pinMode(MOTORPIN3, OUTPUT);
    if (numbers == SIX_Motors) {
	  pinMode(MOTORPIN4, OUTPUT);
      pinMode(MOTORPIN5, OUTPUT);
	  initializeSoftPWM();
    }
  #endif
    
  commandAllMotors(1000);
}

void writeMotors() {
  analogWrite(MOTORPIN0, motorCommand[MOTOR1] / 8);
  analogWrite(MOTORPIN1, motorCommand[MOTOR2] / 8);
  analogWrite(MOTORPIN2, motorCommand[MOTOR3] / 8);
  analogWrite(MOTORPIN3, motorCommand[MOTOR4] / 8); 
  if (numberOfMotors == SIX_Motors) {
	#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	  analogWrite(MOTORPIN4, motorCommand[MOTOR5] / 8);
      analogWrite(MOTORPIN5, motorCommand[MOTOR6] / 8);
	#else
	  atomicPWM_PIN5_highState = motorCommand[MOTOR6]/8;
      atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
      atomicPWM_PIN6_highState = motorCommand[MOTOR5]/8;
      atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
    #endif
  }
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    else if (numberOfMotors == EIGHT_Motors) {
      analogWrite(MOTORPIN4, motorCommand[MOTOR5] / 8);
      analogWrite(MOTORPIN5, motorCommand[MOTOR6] / 8);
      analogWrite(MOTORPIN6, motorCommand[MOTOR7] / 8);
      analogWrite(MOTORPIN7, motorCommand[MOTOR8] / 8);
    }
  #endif
}

void commandAllMotors(int command) {
  analogWrite(MOTORPIN0, command / 8);
  analogWrite(MOTORPIN1, command / 8);
  analogWrite(MOTORPIN2, command / 8);
  analogWrite(MOTORPIN3, command / 8);
  if (numberOfMotors == SIX_Motors) {
	#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	  analogWrite(MOTORPIN4, command / 8);
      analogWrite(MOTORPIN5, command / 8);
	#else
	  atomicPWM_PIN5_highState = command/8;
      atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
      atomicPWM_PIN6_highState = command/8;
      atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
    #endif
	  
  }
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    else if (numberOfMotors == EIGHT_Motors) {
      analogWrite(MOTORPIN4, command / 8);
      analogWrite(MOTORPIN5, command / 8);
      analogWrite(MOTORPIN6, command / 8);
      analogWrite(MOTORPIN7, command / 8);
    }
  #endif
}  


#endif