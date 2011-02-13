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

#ifndef _AQ_RECEIVER_MULTIPILOT_H_
#define _AQ_RECEIVER_MULTIPILOT_H_

#include "Receiver.h"

class ReceiverForMultipilot : public Receiver 
{
private:
  int receiverChannel[LASTCHANNEL];

public:
  ReceiverForMultipilot() : Receiver()
  {
    receiverChannel[ROLL] = ROLLCH;
    receiverChannel[PITCH] = PITCHCH;
    receiverChannel[YAW] = YAWCH;
    receiverChannel[THROTTLE] = THROTTLECH;
    receiverChannel[MODE] = MODECH;
    receiverChannel[AUX] = AUXCH;

  }

  // Configure each receiver pin for PCINT
  void initialize() 
  {
    this->_initialize(); // load in calibration xmitFactor from EEPROM
    ServoDecode.begin();
    ServoDecode.setFailsafe(3,1234); // set channel 3 failsafe pulse  width
  }

  // Calculate PWM pulse width of receiver data
  // If invalid PWM measured, use last known good time
  void read() 
  {
    uint16_t data[6];

    if(ServoDecode.getState()!= READY_state)
    {
      for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      {
        safetyCheck=0;
        data[channel]=5000;
      }
    }
    else
    {
      data[ROLL] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[ROLL]);
      data[PITCH] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[PITCH]);
      data[THROTTLE] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[THROTTLE]);
      data[YAW] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[YAW]);
      data[MODE] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[MODE]);
      data[AUX] = ServoDecode.GetChannelPulseWidth((int)receiverChannel[AUX]);
      safetyCheck=1;
    }


    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
    {
      currentTime = micros();
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * data[channel]) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      previousTime = currentTime;
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
    {
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
      //transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel])) + transmitterZero[channel];
    }
    // No xmitFactor reduction applied for throttle, mode and
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
    {
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
    }

  }
};

#endif