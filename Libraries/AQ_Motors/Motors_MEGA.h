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

//#if defined (USE_400HZ_ESC)
  #define PWM_FREQUENCY 400   // in Hz
//#else
//  #define PWM_FREQUENCY 300   // in Hz
//#endif  

  #define PWM_SERVO_FREQUENCY 50   // in Hz


#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)
#define PWM_SERVO_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_SERVO_FREQUENCY)


void writeMotors()
{

  OCR3B = motorCommand[MOTOR1] * 2 ;
  OCR3C = motorCommand[MOTOR2] * 2 ;
  OCR3A = motorCommand[MOTOR3] * 2 ;
  if (flightConfigType == TRI) {
    OCR4B = motorCommand[MOTOR4] * 2 ;
  }
  else {
    OCR4A = motorCommand[MOTOR4] * 2 ;
    if (numberOfMotors == 6 || numberOfMotors == 8) {
      OCR4B = motorCommand[MOTOR5] * 2 ;
      OCR4C = motorCommand[MOTOR6] * 2 ;
    }
    if (numberOfMotors == 8) {
      OCR1A = motorCommand[MOTOR7] * 2 ;
      OCR1B = motorCommand[MOTOR8] * 2 ;
    }
  }
}
void commandAllMotors(int command) {
  OCR3B = command * 2 ;
  OCR3C = command * 2 ;
  OCR3A = command * 2 ;
  if (flightConfigType == TRI) {
    OCR4B = command * 2 ;
  }
  else {
    OCR4A = command * 2 ;
    if (numberOfMotors == 6 || numberOfMotors == 8) {
      OCR4B = command * 2 ;
      OCR4C = command * 2 ;
    }
    if (numberOfMotors == 8) {
      OCR1A = command * 2 ;
      OCR1B = command * 2 ;
    }
  }
}


void initializeMotors(byte numbers) {

  numberOfMotors = numbers;

  DDRE = DDRE | B00111000;     // Set ports to output PE3-5, OC3A, OC3B, OC3C
  if (flightConfigType == TRI) {
    DDRH = DDRH | B00010000;   // Set port to output PH4, OC4B
  } else {
    DDRH = DDRH | B00001000;   // Set port to output PH3, OC4A
    if (numberOfMotors == 6) { 
      DDRH = DDRH | B00111000;   // Set ports to output PH3-5, OC4A, OC4B, OC4C
    }
    if (numberOfMotors == 8) {
      DDRB = DDRB | B01100000;   // PB5-6, OC1A, OC1B
    }
  }
  // Init PWM Timer 3                                       // WGMn1 WGMn2 WGMn3  = Mode 14 Fast PWM, TOP = ICRn ,Update of OCRnx at BOTTOM
  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  // Clear OCnA/OCnB/OCnC on compare match, set OCnA/OCnB/OCnC at BOTTOM (non-inverting mode)
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);                 // Prescaler set to 8, that gives us a resolution of 0.5us
  ICR3 = PWM_COUNTER_PERIOD;                                // Clock_speed / ( Prescaler * desired_PWM_Frequency) #defined above.
  
  // Init PWM Timer 4 for ESC or servo (on TRI)
  if (flightConfigType == TRI) {
    TCCR4A = (1<<WGM41)|(1<<COM4B1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_SERVO_COUNTER_PERIOD;
  } else {
    TCCR4A = (1<<WGM41)|(1<<COM4A1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
  }
 
  if ((numberOfMotors == 6) || (numberOfMotors == 8)) {
    // Init PWM Timer 4
    TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
    TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
    ICR4 = PWM_COUNTER_PERIOD;
  }
 
  if (numberOfMotors == 8){
    // Init PWM Timer 1
    TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = PWM_COUNTER_PERIOD;
  }

  commandAllMotors(1000);                                     // Initialise motors to 1000us (stopped)

}


#endif