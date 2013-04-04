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

#ifndef _AEROQUAD_RECEIVER_328p_H_
#define _AEROQUAD_RECEIVER_328p_H_

#include "Arduino.h"
#include "Receiver_Base_328p.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000
#define MAX_NO_SIGNAL_COUNTER 10

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
  unsigned int  lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

// Attaches PCINT to Arduino Pin
void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  }
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

// ISR which records time of rising or falling edge of signal
static void measurePulseWidthISR(uint8_t port, uint8_t pinoffset) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  currentTime = micros();
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = pinoffset + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
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

SIGNAL(PCINT0_vect) {
  measurePulseWidthISR(0, 8); // PORT B
}

SIGNAL(PCINT2_vect) {
  measurePulseWidthISR(2, 0); // PORT D
}

// defines arduino pins used for receiver in arduino pin numbering schema
static byte receiverPin[5] = {2, 5, 6, 4, 7}; // pins used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX


void initializeReceiverPWM() {

  for (byte channel = XAXIS; channel < nbReceiverChannel; channel++) {
    pinMode(receiverPin[channel], INPUT);
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
    attachPinChangeInterrupt(receiverPin[channel]);
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
#define SERIAL_SUM_PPM             0,1,3,2,4 // ROLL,PITCH,THR,YAW... For Robe/Hitec/Futaba/Turnigy9xFrsky
#define PPM_PIN_INTERRUPT()          attachInterrupt(0, rxInt, RISING) //PIN 0

static uint8_t rcChannel[MAX_NB_CHANNEL] = {SERIAL_SUM_PPM};
volatile uint16_t rcValue[MAX_NB_CHANNEL] = {1500,1500,1500,1500,1500}; // interval [1000;2000]

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



