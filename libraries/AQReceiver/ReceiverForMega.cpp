/*
  AeroQuad v2.2 - Feburary 2011
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

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include "ReceiverForMega.h"
#include "pins_arduino.h"

#include <AQMath.h>


volatile uint8_t *port_to_pcmask[] = 
{
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};

volatile static uint8_t PCintLast[3];

// Channel data
typedef struct 
{
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;

volatile static tPinTimingData pinData[LASTCHANNEL];

static void MegaPcIntISR() 
{
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
  if ((mask &= PCMSK2) == 0) 
  {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) 
  {
    bit = 0x01 << i;
    if (bit & mask) 
    {
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) 
      {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
        {
          pinData[pin].edge = RISING_EDGE;
        }
        else
        {
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
        }
      }
      else 
      {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) 
        {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}

SIGNAL(PCINT2_vect) 
{
  MegaPcIntISR();
}

#ifdef AeroQuadMega_v1
  // arduino pins 67, 65, 64, 66, 63, 62
  static byte receiverPin[6] = {5, 3, 2, 4, 1, 0}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#else
 //arduino pins 63, 64, 65, 62, 66, 67
  static byte receiverPin[6] = {1, 2, 3, 0, 4, 5}; // bit number of PORTK used for ROLL, PITCH, YAW, THROTTLE, MODE, AUX
#endif

void ReceiverForMega::initialize() 
{
  this->_initialize(); // load in calibration xmitFactor from EEPROM
  DDRK = 0;
  PORTK = 0;
  PCMSK2 |= 0x3F;
  PCICR |= 0x1 << 2;

  for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
  {
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
  }
}

// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
void ReceiverForMega::read() 
{
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
  {
    byte pin = receiverPin[channel];
    uint8_t oldSREG = SREG;
    cli();
    // Get receiver value read by pin change interrupt handler
    uint16_t lastGoodWidth = pinData[pin].lastGoodWidth;
    SREG = oldSREG;

    // Apply transmitter calibration adjustment
    _receiverData[channel] = (_mTransmitter[channel] * lastGoodWidth) + _bTransmitter[channel];
    // Smooth the flight control transmitter inputs
    _transmitterCommandSmooth[channel] = filterSmooth(_receiverData[channel], _transmitterCommandSmooth[channel], _transmitterSmooth[channel]);
  }

  // Reduce transmitter commands using xmitFactor and center around 1500
  for (byte channel = ROLL; channel < THROTTLE; channel++)
  {
    _transmitterCommand[channel] = ((_transmitterCommandSmooth[channel] - _transmitterZero[channel]) * _xmitFactor) + _transmitterZero[channel];
  }
  // No xmitFactor reduction applied for throttle, mode and AUX
  for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
  {
    _transmitterCommand[channel] = _transmitterCommandSmooth[channel];
  }
}

#endif
