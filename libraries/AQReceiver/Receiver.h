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

#include "WProgram.h"

#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define LASTCHANNEL 6

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000



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
  Receiver();

  // ******************************************************************
  // The following function calls must be defined in any new subclasses
  // ******************************************************************
  virtual void initialize();
  virtual void read();

  // **************************************************************
  // The following functions are common between all Gyro subclasses
  // **************************************************************

  void _initialize();
  const int getRaw(byte channel);
  const int getData(byte channel);
  const int getTrimData(byte channel);
  const int getZero(byte channel);
  void setZero(byte channel, int value);
  const int getTransmitterTrim(byte channel);
  void setTransmitterTrim(byte channel, int value);
  const float getSmoothFactor(byte channel);
  void setSmoothFactor(byte channel, float value);
  const float getXmitFactor();
  void setXmitFactor(float value);
  const float getTransmitterSlope(byte channel);
  void setTransmitterSlope(byte channel, float value);
  const float getTransmitterOffset(byte channel);
  void setTransmitterOffset(byte channel, float value);
  const float getAngle(byte channel);
};

#endif