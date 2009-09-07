/*
  AeroQuad v1.3.1 - September 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
  
  Interrupt based method inspired by Dror Caspi
  http://www.rcgroups.com/forums/showpost.php?p=12356667&postcount=1639
  
  Version in AeroQuad code is written by Ted Carancho
 
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

#include "pins_arduino.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINWIDTH 900
#define MAXWIDTH 2100


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
  unsigned long lastGoodWidth;
} pinTimingData;  

volatile static pinTimingData pinData[24]; 

// Attaches PCINT to Arduino Pin
void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
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

// Detaches PCINT from Arduino Pin
void detachPinChangeInterrupt(uint8_t pin) {
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

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

// ISR which records time of rising or falling edge of signal
static void measurePulseWidthISR(uint8_t port) {
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
      pin = port * 8 + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
        pinData[pin].riseTime = currentTime;
        pinData[pin].edge = RISING_EDGE;
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        if ((time >= MINWIDTH) && (time <= MAXWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          //pinData[pin].fallTime = currentTime;
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        } 
      }
    }
  }
}

SIGNAL(PCINT0_vect) {
  measurePulseWidthISR(0);
}
SIGNAL(PCINT1_vect) {
  measurePulseWidthISR(1);
}
SIGNAL(PCINT2_vect) {
  measurePulseWidthISR(2);
}

// Configure each receiver pin for PCINT
void configureReceiver() {
  for (channel = ROLL; channel < LASTCHANNEL; channel++) {
    pinMode(channel, INPUT);
    attachPinChangeInterrupt(receiverChannel[channel]);
    pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  }
}

// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
unsigned int readReceiver(byte receiverPin) {
  return pinData[receiverPin].lastGoodWidth;
}
