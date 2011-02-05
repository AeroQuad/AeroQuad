/*
  AeroQuad v2.1 - January 2011
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

#ifndef _RECEIVER_328_H_
#define _RECEIVER_328_H_

#include <Receiver.h>

/*************************************************/
/*************** AeroQuad PCINT ******************/
/*************************************************/
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
static byte receiverPin[6] = {2, 5, 6, 4, 7, 8}; // pins used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX

class Receiver328 : public Receiver {
public:
  // Configure each receiver pin for PCINT
  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      pinMode(receiverPin[channel], INPUT);
      pinData[receiverPin[channel]].edge = FALLING_EDGE;
      attachPinChangeInterrupt(receiverPin[channel]);
    }
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      byte pin = receiverPin[channel];
      uint8_t oldSREG = SREG;
      cli();
      // Get receiver value read by pin change interrupt handler
      uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
      SREG = oldSREG;

      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * lastGoodWidth) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};

#endif