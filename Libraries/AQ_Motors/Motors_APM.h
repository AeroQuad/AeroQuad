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


#ifndef _AEROQUAD_MOTORS_APM_H_
#define _AEROQUAD_MOTORS_APM_H_

#include <WProgram.h>

#include "Motors.h"


class Motors_APM : public Motors {
public:

  Motors_APM() {
  }

  void initialize(NB_Motors numbers) {
    // Init PWM Timer 1
    //pinMode(11,OUTPUT); //     (PB5/OC1A)
    pinMode(12,OUTPUT); //OUT2 (PB6/OC1B)
    pinMode(13,OUTPUT); //OUT3 (PB7/OC1C)

    //Remember the registers not declared here remains zero by default...
    TCCR1A =((1<<WGM11)|(1<<COM1B1)|(1<<COM1C1)); //Please read page 131 of DataSheet, we are changing the registers settings of WGM11,COM1B1,COM1A1 to 1 thats all...
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11); //Prescaler set to 8, that give us a resolution of 0.5us, read page 134 of data sheet
    //OCR1A = 3000; //PB5, none
    OCR1B = 2000; //PB6, OUT2
    OCR1C = 2000; //PB7  OUT3
    ICR1 = 6600; //300hz freq...

    // Init PWM Timer 3
    pinMode(2,OUTPUT); //OUT7 (PE4/OC3B)
    pinMode(3,OUTPUT); //OUT6 (PE5/OC3C)
    //pinMode(5,OUTPUT); //     (PE3/OC3A)
    TCCR3A =((1<<WGM31)|(1<<COM3B1)|(1<<COM3C1));
    TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
    //OCR3A = 3000; //PE3, NONE
    OCR3B = 2000; //PE4, OUT7
    OCR3C = 2000; //PE5, OUT6
    ICR3 = 40000; //50hz freq (standard servos)

    // Init PWM Timer 5
    pinMode(44,OUTPUT);  //OUT1 (PL5/OC5C)
    pinMode(45,OUTPUT);  //OUT0 (PL4/OC5B)
    //pinMode(46,OUTPUT);  //     (PL3/OC5A)

    TCCR5A =((1<<WGM51)|(1<<COM5B1)|(1<<COM5C1));
    TCCR5B = (1<<WGM53)|(1<<WGM52)|(1<<CS51);
    //OCR5A = 3000; //PL3,
    OCR5B = 2000; //PL4, OUT0
    OCR5C = 2000; //PL5, OUT1
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
    OCR4B = 2000; //PH4, OUT5
    OCR4C = 2000; //PH5, OUT4

    //TCCR4B |=(1<<ICES4); //Changing edge detector (rising edge).
    //TCCR4B &=(~(1<<ICES4)); //Changing edge detector. (falling edge)
    TIMSK4 |= (1<<ICIE4); // Enable Input Capture interrupt. Timer interrupt mask
    
    commandAllMotors(1000);
  }

  void write() {
    OCR1B = motorCommand[MOTOR1] * 2;
    OCR1C = motorCommand[MOTOR2] * 2;
    OCR5B = motorCommand[MOTOR3] * 2;
    OCR5C = motorCommand[MOTOR4] * 2;
  }

  void commandAllMotors(int command) {
    OCR1B = command * 2;
    OCR1C = command * 2;
    OCR5B = command * 2;
    OCR5C = command * 2;
  }
  
};

#endif