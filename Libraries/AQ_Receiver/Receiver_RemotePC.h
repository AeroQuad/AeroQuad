/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AEROQUAD_RECEIVER_REMOTE_PC_H_
#define _AEROQUAD_RECEIVER_REMOTE_PC_H_

#include "Arduino.h"
#include "Receiver.h"

void initializeReceiver(int nbChannel) {

  initializeReceiverParam(nbChannel);
  for (byte channel = XAXIS; channel < THROTTLE; channel++) {
    receiverCommand[channel] = 1500;
    receiverZero[channel] = 1500;
  }
  receiverCommand[THROTTLE] = 0;
  receiverZero[THROTTLE] = 0;
  receiverCommand[MODE] = 2000;
  receiverZero[MODE] = 0;
  receiverCommand[AUX] = 2000;
  receiverZero[AUX] = 0;
}

int getRawChannelValue(byte channel) {
  return receiverCommand[channel];
}
  
void setChannelValue(byte channel,int value) {
  receiverCommand[channel] = value;
}


#endif



