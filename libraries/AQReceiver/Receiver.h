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

#ifndef _RECEIVER_H_
#define _RECEIVER_H_

class  Receiver {
public:
  int receiverData[LASTCHANNEL];
  int transmitterCommand[LASTCHANNEL];
  int transmitterCommandSmooth[LASTCHANNEL];
  int transmitterZero[3];
  int transmitterTrim[3];
  // Controls the strength of the commands sent from the transmitter
  // xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
  float xmitFactor; // Read in from EEPROM
  float transmitterSmooth[LASTCHANNEL];
  float mTransmitter[LASTCHANNEL];
  float bTransmitter[LASTCHANNEL];
  unsigned long currentTime, previousTime;

  Receiver(void) {
    transmitterCommand[ROLL] = 1500;
    transmitterCommand[PITCH] = 1500;
    transmitterCommand[YAW] = 1500;
    transmitterCommand[THROTTLE] = 1000;
    transmitterCommand[MODE] = 1000;
    transmitterCommand[AUX] = 1000;

    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterCommandSmooth[channel] = 1.0;
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterZero[channel] = 1500;
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize(void);
  virtual void read(void);

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************

  void _initialize(void) {
    xmitFactor = readFloat(XMITFACTOR_ADR);

    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
      mTransmitter[channel] = readFloat(offset+0);
      bTransmitter[channel] = readFloat(offset+4);
      transmitterSmooth[channel] = readFloat(offset+8);
    }
  }

  const int getRaw(byte channel) {
    return receiverData[channel];
  }

  const int getData(byte channel) {
    // reduce sensitivity of transmitter input by xmitFactor
    return transmitterCommand[channel];
  }

  const int getTrimData(byte channel) {
    return receiverData[channel] - transmitterTrim[channel];
  }

  const int getZero(byte channel) {
    return transmitterZero[channel];
  }

  void setZero(byte channel, int value) {
    transmitterZero[channel] = value;
  }

  const int getTransmitterTrim(byte channel) {
    return transmitterTrim[channel];
  }

  void setTransmitterTrim(byte channel, int value) {
    transmitterTrim[channel] = value;
  }

  const float getSmoothFactor(byte channel) {
    return transmitterSmooth[channel];
  }

  void setSmoothFactor(byte channel, float value) {
    transmitterSmooth[channel] = value;
  }

  const float getXmitFactor(void) {
    return xmitFactor;
  }

  void setXmitFactor(float value) {
    xmitFactor = value;
  }

  const float getTransmitterSlope(byte channel) {
    return mTransmitter[channel];
  }

  void setTransmitterSlope(byte channel, float value) {
    mTransmitter[channel] = value;
  }

  const float getTransmitterOffset(byte channel) {
    return bTransmitter[channel];
  }

  void setTransmitterOffset(byte channel, float value) {
    bTransmitter[channel] = value;
  }

  const float getAngle(byte channel) {
    // Scale 1000-2000 usecs to -45 to 45 degrees
    // m = 0.09, b = -135
    // reduce transmitterCommand by xmitFactor to lower sensitivity of transmitter input
    return (0.09 * transmitterCommand[channel]) - 135;
  }
};

#endif