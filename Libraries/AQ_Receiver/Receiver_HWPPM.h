	/*
  AeroQuad v3.0 - May 2011
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

/* This code will need a HW modification on v2 shield, see AQ forums
   http://aeroquad.com/showthread.php?4239-Timer-based-PPM-sum-input-on-mega-%28v3-SW%29&highlight=jitter

   In short it will expect to receive cPPM signal at ICP5 pin. 
*/

#ifndef _AEROQUAD_RECEIVER_HWPPM_H_
#define _AEROQUAD_RECEIVER_HWPPM_H_

#if !defined (__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Receiver_HWPPM can only be used on ATMega
#endif

#include <Arduino.h>
#include "Receiver.h"
#include "pins_arduino.h"
#include <avr/interrupt.h>
#include <AQMath.h>
#include "GlobalDefined.h"

// Channel data
volatile unsigned int startPulse = 0;
volatile byte         ppmCounter = 8; // ignore data until first sync pulse
volatile int          PWM_RAW[8] = { 2200,2200,2200,2200,2200,2200,2200,2200 };

#define TIMER5_FREQUENCY_HZ 50
#define TIMER5_PRESCALER    8
#define TIMER5_PERIOD       (F_CPU/TIMER5_PRESCALER/TIMER5_FREQUENCY_HZ)

/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER5_CAPT_vect)//interrupt.
{
  if ((1 << ICES5) & TCCR5B) {
    // Triggered at rising edge
    startPulse = ICR5;         // Save time at pulse start
  }
  else {
    // Triggered at dropping edge; measure pulse length
    unsigned int stopPulse = ICR5;
    // Note: compensate for timer overflow if needed
    unsigned int pulseWidth = ((startPulse > stopPulse) ? TIMER5_PERIOD : 0) + stopPulse - startPulse;

    if (pulseWidth > 5000) {      // Verify if this is the sync pulse
      ppmCounter = 0;             // -> restart the channel counter
    }
    else {
      if (ppmCounter < 8) { // channels 9- will get ignored here
        PWM_RAW[ppmCounter] = pulseWidth; // Store measured pulse length
        ppmCounter++;                     // Advance to next channel
      }
    }
  }
  TCCR5B ^= (1 << ICES5); // Switch edge
}

#define SERIAL_SUM_PPM_1         1,2,3,0,4,5,6,7 // PITCH,YAW,THR,ROLL... For Graupner/Spektrum
#define SERIAL_SUM_PPM_2         0,1,3,2,4,5,6,7 // ROLL,PITCH,THR,YAW... For Robe/Hitec/Futaba
#define SERIAL_SUM_PPM_3         1,0,3,2,4,5,6,7 // PITCH,ROLL,THR,YAW... For some Hitec/Sanwa/Others

#if defined (SKETCH_SERIAL_SUM_PPM)
  #define SERIAL_SUM_PPM SKETCH_SERIAL_SUM_PPM
#else	
  #define SERIAL_SUM_PPM SERIAL_SUM_PPM_1
#endif

static uint8_t rcChannel[8] = {SERIAL_SUM_PPM};

void initializeReceiver(int nbChannel) {

  initializeReceiverParam(nbChannel);
  pinMode(48, INPUT); // ICP5
  pinMode(A8, INPUT); // this is the original location of the first RX channel

  // Configure timer HW
  TCCR5A = ((1<<WGM50)|(1<<WGM51)|(1<<COM5C1)|(1<<COM5B1)|(1<<COM5A1));
  TCCR5B = ((1<<WGM52)|(1<<WGM53)|(1<<CS51)|(1<<ICES5)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR5A = TIMER5_PERIOD; 

  TIMSK5 |= (1<<ICIE5); //Timer interrupt mask
}


int getRawChannelValue(byte channel) {
  uint8_t oldSREG = SREG;
  cli(); // Disable interrupts to prevent race with ISR updating PWM_RAW

  // Apply receiver calibration adjustment
  int receiverRawValue = ((PWM_RAW[rcChannel[channel]]+800)/2);

  SREG = oldSREG;
  
  return receiverRawValue;
}

#endif



