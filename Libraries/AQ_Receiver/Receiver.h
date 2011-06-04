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
  Receiver() {

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
	
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      mTransmitter[channel] = 1;
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      bTransmitter[channel] = 1;
    for (byte channel = ROLL; channel < LASTCHANNEL; channel++)
      transmitterSmooth[channel] = 1;
  }


  virtual void initialize(void) {}
  virtual void read(void) {}
  

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

  const float getSmoothFactor(byte channel) {
    return transmitterSmooth[channel];
  }

  void setSmoothFactor(byte channel, float value) {
    transmitterSmooth[channel] = value;
  }

  const int getTransmitterTrim(byte channel) {
    return transmitterTrim[channel];
  }

  void setTransmitterTrim(byte channel, int value) {
    transmitterTrim[channel] = value;
  }

  // return the smoothed & scaled number of radians/sec in stick movement - zero centered
  const float getSIData(byte channel) {
    return ((transmitterCommand[channel] - transmitterZero[channel]) * (2.5 * PWM2RAD));  // +/- 2.5RPS 50% of full rate
  }

  // returns Zero value of channel in PWM
  const int getZero(byte channel) {
    return transmitterZero[channel];
  }
  
  // sets zero value of channel in PWM  
  void setZero(byte channel, int value) {
    transmitterZero[channel] = value;
  }

  // returns smoothed & scaled receiver(channel) in PWM values, zero centered
  const int getData(byte channel) {
    return transmitterCommand[channel];
  }
};
#endif



