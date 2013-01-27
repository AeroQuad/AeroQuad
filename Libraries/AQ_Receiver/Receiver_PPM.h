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

#ifndef _AEROQUAD_RECEIVER_PPM_H_
#define _AEROQUAD_RECEIVER_PPM_H_

#if defined (__AVR_ATmega328P__) || defined(__AVR_ATmegaUNO__)
  #define PPM_PIN_INTERRUPT()          attachInterrupt(0, rxInt, RISING) //PIN 0
#else
  #define PPM_PIN_INTERRUPT()          attachInterrupt(4, rxInt, RISING) //PIN 19, also used for Spektrum satellite option
#endif

#include "Arduino.h"
#include "Receiver.h"

#include "pins_arduino.h"
#include <AQMath.h>
#include "GlobalDefined.h"

#include "Receiver_PPM_common.h"

static uint8_t rcChannel[PPM_CHANNELS] = {SERIAL_SUM_PPM};
volatile uint16_t rcValue[PPM_CHANNELS] = {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

static void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = PPM_CHANNELS;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>3000) { 
    chan = 0;
  }
  else if( 800 < diff && diff < 2200 && chan < PPM_CHANNELS ) {
    rcValue[chan] = diff;
    chan++;
  }
  else {
    chan = PPM_CHANNELS;
  }
}

void initializeReceiver(int nbChannel) {

  initializeReceiverParam(nbChannel);
  PPM_PIN_INTERRUPT();
}

int getRawChannelValue(byte channel) {
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts

  int rawChannelValue = rcValue[rcChannel[channel]];
  SREG = oldSREG;
  
  return rawChannelValue;
}


#endif
