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

#ifndef _AQ_RECEIVER_H_
#define _AQ_RECEIVER_H_

class Receiver 
{
private:
  int _transmitterTrim[3];
  // Controls the strength of the commands sent from the transmitter
  // xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)

protected:
  int _transmitterCommand[LASTCHANNEL];
  int _receiverData[LASTCHANNEL];
  int _transmitterCommandSmooth[LASTCHANNEL];
  int _transmitterZero[3];
  float _mTransmitter[LASTCHANNEL];
  float _bTransmitter[LASTCHANNEL];  
  float _xmitFactor; // Read in from EEPROM
  float _transmitterSmooth[LASTCHANNEL];
  unsigned long _currentTime;
  unsigned long _previousTime;
  
public:
  Receiver() 
  {
    _transmitterCommand[ROLL] = 1500;
    _transmitterCommand[PITCH] = 1500;
    _transmitterCommand[YAW] = 1500;
    _transmitterCommand[THROTTLE] = 1000;
    _transmitterCommand[MODE] = 1000;
    _transmitterCommand[AUX] = 1000;

    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
    {
      _transmitterCommandSmooth[channel] = 1.0;
    }
    for (byte channel = ROLL; channel < THROTTLE; channel++)
    {
      _transmitterZero[channel] = 1500;
    }
  }

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize();
  virtual void read();

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************

  void _initialize() 
  {
    _xmitFactor = readFloat(XMITFACTOR_ADR);

    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
    {
      byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
      _mTransmitter[channel] = readFloat(offset+0);
      _bTransmitter[channel] = readFloat(offset+4);
      _transmitterSmooth[channel] = readFloat(offset+8);
    }
  }

  const int getRaw(byte channel) 
  {
    return _receiverData[channel];
  }

  const int getData(byte channel) 
  {
    // reduce sensitivity of transmitter input by xmitFactor
    return _transmitterCommand[channel];
  }

  const int getTrimData(byte channel) 
  {
    return _receiverData[channel] - _transmitterTrim[channel];
  }

  const int getZero(byte channel) 
  {
    return _transmitterZero[channel];
  }

  void setZero(byte channel, int value) 
  {
    _transmitterZero[channel] = value;
  }

  const int getTransmitterTrim(byte channel) 
  {
    return _transmitterTrim[channel];
  }

  void setTransmitterTrim(byte channel, int value) 
  {
    _transmitterTrim[channel] = value;
  }

  const float getSmoothFactor(byte channel) 
  {
    return _transmitterSmooth[channel];
  }

  void setSmoothFactor(byte channel, float value) 
  {
    _transmitterSmooth[channel] = value;
  }

  const float getXmitFactor() 
  {
    return _xmitFactor;
  }

  void setXmitFactor(float value) 
  {
    _xmitFactor = value;
  }

  const float getTransmitterSlope(byte channel) 
  {
    return _mTransmitter[channel];
  }

  void setTransmitterSlope(byte channel, float value) 
  {
    _mTransmitter[channel] = value;
  }

  const float getTransmitterOffset(byte channel) 
  {
    return _bTransmitter[channel];
  }

  void setTransmitterOffset(byte channel, float value) 
  {
    _bTransmitter[channel] = value;
  }

  const float getAngle(byte channel) 
  {
    // Scale 1000-2000 usecs to -45 to 45 degrees
    // m = 0.09, b = -135
    // reduce transmitterCommand by xmitFactor to lower sensitivity of transmitter input
    return (0.09 * _transmitterCommand[channel]) - 135;
  }
};

#endif