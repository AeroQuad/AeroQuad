	/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AEROQUAD_RECEIVER_H_
#define _AEROQUAD_RECEIVER_H_

#include <WProgram.h>

#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF 100
#define LASTCHANNEL 6
//int delta;


class Receiver {
protected:
  float xmitFactor;
  int receiverData[LASTCHANNEL];
  int transmitterTrim[3];
  int transmitterZero[3];
  int transmitterCommand[LASTCHANNEL];
  int transmitterCommandSmooth[LASTCHANNEL];
  float mTransmitter[LASTCHANNEL];
  float bTransmitter[LASTCHANNEL];
  float transmitterSmooth[LASTCHANNEL];
  
public:  
  Receiver();

  virtual void initialize(void) {}
  virtual void read(void) {}
  
  const float getXmitFactor(void);
  void setXmitFactor(float xmitFactor);
  const float getTransmitterSlope(byte channel);
  void setTransmitterSlope(byte channel, float value);
  const float getTransmitterOffset(byte channel);
  void setTransmitterOffset(byte channel, float value);
  const float getSmoothFactor(byte channel);
  void setSmoothFactor(byte channel, float value);  
  const int getTransmitterTrim(byte channel);
  void setTransmitterTrim(byte channel, int value);
  const float getSIData(byte channel);
  const int getZero(byte channel);  
  void setZero(byte channel, int value);
  const int getData(byte channel);
};
#endif



