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

#ifndef _RECEIVER_1280_H_
#define _RECEIVER_1280_H_

#include <Receiver.h>

/******************************************************/
/*************** AeroQuad Mega PCINT ******************/
/******************************************************/
class Receiver1280Fake :
public Receiver {
private:

public:
  Receiver1280Fake() :
  Receiver(){
  }

  void initialize() {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    DDRK = 0;
    PORTK = 0;
    PCMSK2 |= 0x3F;
    PCICR |= 0x1 << 2;
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read(void) {
    uint16_t data[6];
    uint8_t oldSREG;

    oldSREG = SREG;
    cli();
    // Buffer receiver values read from pin change interrupt handler
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      data[channel] = 1500;
    SREG = oldSREG;

    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      currentTime = micros();
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      //transmitterCommandSmooth[channel] = transmitterFilter[channel].filter(receiverData[channel]);
      previousTime = currentTime;
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and AUX
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};

#endif