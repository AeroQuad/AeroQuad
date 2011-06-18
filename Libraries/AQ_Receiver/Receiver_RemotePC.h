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

#ifndef _AEROQUAD_RECEIVER_REMOTE_PC_H_
#define _AEROQUAD_RECEIVER_REMOTE_PC_H_

#include <WProgram.h>
#include "Receiver.h"

class Receiver_RemotePC : public Receiver {
public:  
  Receiver_RemotePC() {
  }

  void initialize(void) {
	for (byte channel = ROLL; channel < THROTTLE; channel++) {
	  transmitterCommand[channel] = 1500;
      transmitterZero[channel] = 1500;
	}
    transmitterCommand[THROTTLE] = 0;
    transmitterZero[THROTTLE] = 0;
	transmitterCommand[MODE] = 2000;
	transmitterZero[MODE] = 0;
	transmitterCommand[AUX] = 2000;
	transmitterZero[AUX] = 0;
  }

  void read(void) {
    // do nothing here
  }
  
  void setChannelValue(byte channel,int value) {
	transmitterCommand[channel] = value;
  }
};

#endif



