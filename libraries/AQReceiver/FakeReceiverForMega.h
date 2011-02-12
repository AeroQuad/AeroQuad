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

#ifndef _AQ_FAKE_RECEIVER_MEGA_H_
#define _AQ_FAKE_RECEIVER_MEGA_H_

#include "Receiver.h"

class FakeReceiverForMega : public Receiver 
{
private:

public:
  FakeReceiverForMega() :
  Receiver()
  {
  }

  void initialize() 
  {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    DDRK = 0;
    PORTK = 0;
    PCMSK2 |= 0x3F;
    PCICR |= 0x1 << 2;
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read() 
  {
    uint16_t data[6];
    uint8_t oldSREG;

    oldSREG = SREG;
    cli();
    // Buffer receiver values read from pin change interrupt handler
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      data[channel] = 1500;
    SREG = oldSREG;

    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
    {
      _currentTime = micros();
      // Apply transmitter calibration adjustment
      _receiverData[channel] = (_mTransmitter[channel] * data[channel]) + _bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      _transmitterCommandSmooth[channel] = filterSmooth(_receiverData[channel], _transmitterCommandSmooth[channel], _transmitterSmooth[channel]);
      //transmitterCommandSmooth[channel] = transmitterFilter[channel].filter(receiverData[channel]);
      _previousTime = _currentTime;
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
};

#endif
