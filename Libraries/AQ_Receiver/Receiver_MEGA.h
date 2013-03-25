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

#ifndef _AEROQUAD_RECEIVER_MEGA_H_
#define _AEROQUAD_RECEIVER_MEGA_H_

#include "Arduino.h"
#include "Receiver_Base_MEGA.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000
//#define MAX_NO_SIGNAL_COUNTER 10

#include "pins_arduino.h"
#include <AQMath.h>
#include "GlobalDefined.h"


volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};
volatile static uint8_t PCintLast[3];
// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

static void MegaPcIntISR() {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK2) == 0) {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}

SIGNAL(PCINT2_vect) {
  MegaPcIntISR();
}

//arduino pins 63, 64, 65, 62, 66, 67
static byte receiverPin[12] = {1, 2, 3, 0, 4, 5, 6, 7, 8, 9, 10, 11}; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX


void initializeReceiverPWM() {

  DDRK = 0;
  PORTK = 0;
  PCMSK2 |=(1<<lastReceiverChannel)-1;
  PCICR |= 0x1 << 2;

  for (byte channel = XAXIS; channel < lastReceiverChannel; channel++) {
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
  }
}

int getRawChannelValuePWM(byte channel) {
  byte pin = receiverPin[channel];
  uint8_t oldSREG = SREG;
  cli();
  // Get receiver value read by pin change interrupt handler
  uint16_t receiverRawValue = pinData[pin].lastGoodWidth;
  SREG = oldSREG;
  
  return receiverRawValue;
}



//
// PPM receiver function definition
//
#define SERIAL_SUM_PPM               0,1,3,2,4,5,6,7,8,9,10,11 // ROLL,PITCH,THR,YAW... For Robe/Hitec/Futaba/Turnigy9xFrsky
#define PPM_PIN_INTERRUPT()          attachInterrupt(4, rxInt, RISING) //PIN 19, also used for Spektrum satellite option

static uint8_t rcChannel[MAX_NB_CHANNEL] = {SERIAL_SUM_PPM};
volatile uint16_t rcValue[MAX_NB_CHANNEL] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};

static void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = MAX_NB_CHANNEL;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>3000) { 
    chan = 0;
  }
  else if( 800 < diff && diff < 2200 && chan < MAX_NB_CHANNEL ) {
    rcValue[chan] = diff;
    chan++;
  }
  else {
    chan = MAX_NB_CHANNEL;
  }
}

void initializeReceiverPPM() {

  PPM_PIN_INTERRUPT();
}

int getRawChannelValuePPM(byte channel) {
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts

  int rawChannelValue = rcValue[rcChannel[channel]];
  SREG = oldSREG;
  
  return rawChannelValue;
}

#endif



