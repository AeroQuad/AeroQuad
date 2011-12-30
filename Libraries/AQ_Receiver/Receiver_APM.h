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

#ifndef _AEROQUAD_RECEIVER_APM_H_
#define _AEROQUAD_RECEIVER_APM_H_

#include "Arduino.h"
#include "Receiver.h"
#include <APM_RC.h>

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include <AQMath.h>
#include "GlobalDefined.h"
#include <APM_RC.h>

byte receiverPin[8] = {0,1,3,2,4,5,6,7};
  
void initializeReceiver(int nbChannel = 6) {

  initializeReceiverParam(nbChannel);
}


int getRawChannelValue(byte channel) {
  return readReceiverChannel(receiverPin[channel]);
}

void setChannelValue(byte channel,int value) {
}
  
#endif

#endif


