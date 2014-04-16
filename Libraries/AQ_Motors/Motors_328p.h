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


#ifndef _AEROQUAD_MOTORS_328p_H_
#define _AEROQUAD_MOTORS_328p_H_

#include "Arduino.h"

#include "Motors.h"

#define MOTORPIN0    3
#define MOTORPIN1    9
#define MOTORPIN2   10
#define MOTORPIN3   11
#define MOTORPIN4    5   
#define MOTORPIN5    6  

#define DIGITAL_SERVO_TRI_PINMODE  pinMode(3,OUTPUT); //also right servo for BI COPTER
#define DIGITAL_SERVO_TRI_HIGH     PORTD |= 1<<3;
#define DIGITAL_SERVO_TRI_LOW      PORTD &= ~(1<<3);

volatile uint8_t atomicServo = 125;
  
volatile uint8_t atomicPWM_PIN5_lowState = 0;
volatile uint8_t atomicPWM_PIN5_highState = 0;
volatile uint8_t atomicPWM_PIN6_lowState = 0;
volatile uint8_t atomicPWM_PIN6_highState = 0;

void initializeServo() {
  DIGITAL_SERVO_TRI_PINMODE
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
}
  
void initializeSoftPWM() {
  TCCR0A = 0; // normal counting mode
  TIMSK0 |= (1<<OCIE0A); // Enable CTC interrupt
  TIMSK0 |= (1<<OCIE0B);
}

// 6 motors ISR
ISR(TIMER0_COMPA_vect) {
  static uint8_t state = 0;
  if (numberOfMotors == 6) {
    if (state == 0) {
      PORTD |= 1<<5; //digital PIN 5 high
      OCR0A+= atomicPWM_PIN5_highState; //250 x 4 microsecons = 1ms
      state = 1;
    } 
    else if (state == 1) {
      OCR0A+= atomicPWM_PIN5_highState;
      state = 2;
    } 
    else if (state == 2) {
      PORTD &= ~(1<<5); //digital PIN 5 low
      OCR0A+= atomicPWM_PIN5_lowState;
      state = 0;
    }
  }
  else if (flightConfigType == TRI) {
    static uint8_t count;
    if (state == 0) {
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
}

// 6 motors ISR
ISR(TIMER0_COMPB_vect) { //the same with digital PIN 6 and OCR0B counter
  static uint8_t state = 0;
  if (numberOfMotors == 6) {
    if (state == 0) {
      PORTD |= 1<<6;OCR0B+= atomicPWM_PIN6_highState;state = 1;
    } 
    else if (state == 1) {
      OCR0B+= atomicPWM_PIN6_highState;state = 2;
    } 
    else if (state == 2) {
      PORTD &= ~(1<<6);OCR0B+= atomicPWM_PIN6_lowState;state = 0;
    }
  }
}



void initializeMotors(byte numbers) {

  numberOfMotors = numbers;
  if (flightConfigType == TRI) {
    initializeServo();
  }
  
  pinMode(MOTORPIN0, OUTPUT);
  pinMode(MOTORPIN1, OUTPUT);
  pinMode(MOTORPIN2, OUTPUT);
  pinMode(MOTORPIN3, OUTPUT);
  if (numbers == 6) {
    pinMode(MOTORPIN4, OUTPUT);
    pinMode(MOTORPIN5, OUTPUT);
    initializeSoftPWM();
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
  if (numberOfMotors == 6) {
    atomicPWM_PIN5_highState = motorCommand[MOTOR6]/8;
    atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
    atomicPWM_PIN6_highState = motorCommand[MOTOR5]/8;
    atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
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
  if (numberOfMotors == 6) {
    atomicPWM_PIN5_highState = command/8;
    atomicPWM_PIN5_lowState = 255-atomicPWM_PIN5_highState;
    atomicPWM_PIN6_highState = command/8;
    atomicPWM_PIN6_lowState = 255-atomicPWM_PIN6_highState;
  }
}  


#endif