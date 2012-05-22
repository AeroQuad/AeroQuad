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

#ifndef _AEROQUAD_RECEIVER_SBUS_H_
#define _AEROQUAD_RECEIVER_SBUS_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include "Arduino.h"
#include "Receiver.h"

#include "pins_arduino.h"
#include <AQMath.h>
#include "GlobalDefined.h"

static unsigned int rcValue[18] = {0};
static unsigned int sbusIndex = 0;
#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!

// for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by AQ, but it should be easy to integrate them.
static unsigned int rcChannel[18] = {XAXIS,YAXIS,THROTTLE,ZAXIS,MODE,AUX1,AUX2,AUX3,8,9,10,11,12,13,14,15,16,17};

void initializeReceiver(int nbChannel = 8) {
	initializeReceiverParam(nbChannel);
    Serial1.begin(100000);
}

void readSBUS() {
	static byte sbus[25]={0};
	while(Serial1.available()){
		int val = Serial1.read();
		if(sbusIndex==0 && val != SBUS_SYNCBYTE)
			continue;
		sbus[sbusIndex++] = val;
		if(sbusIndex==25){
			sbusIndex=0;
			rcValue[0]  = ((sbus[1]|sbus[2]<<8) & 0x07FF);
			rcValue[1]  = ((sbus[2]>>3|sbus[3]<<5) & 0x07FF);
			rcValue[2]  = ((sbus[3]>>6|sbus[4]<<2|sbus[5]<<10) & 0x07FF);
			rcValue[3]  = ((sbus[5]>>1|sbus[6]<<7) & 0x07FF);
			rcValue[4]  = ((sbus[6]>>4|sbus[7]<<4) & 0x07FF);
			rcValue[5]  = ((sbus[7]>>7|sbus[8]<<1|sbus[9]<<9) & 0x07FF);
			rcValue[6]  = ((sbus[9]>>2|sbus[10]<<6) & 0x07FF);
			rcValue[7]  = ((sbus[10]>>5|sbus[11]<<3) & 0x07FF);// & the other 8 + 2 channels if you need them
			
			//The following lines: If you need more than 8 channels, max 16 analog + 2 digital.
			//rcValue[8]  = ((sbus[12]|sbus[13]<< 8) & 0x07FF)/2+976; 
			//rcValue[9]  = ((sbus[13]>>3|sbus[14]<<5) & 0x07FF)/2+976; 
			//rcValue[10] = ((sbus[14]>>6|sbus[15]<<2|sbus[16]<<10) & 0x07FF)/2+976; 
			//rcValue[11] = ((sbus[16]>>1|sbus[17]<<7) & 0x07FF)/2+976; 
			//rcValue[12] = ((sbus[17]>>4|sbus[18]<<4) & 0x07FF)/2+976; 
			//rcValue[13] = ((sbus[18]>>7|sbus[19]<<1|sbus[20]<<9) & 0x07FF)/2+976; 
			//rcValue[14] = ((sbus[20]>>2|sbus[21]<<6) & 0x07FF)/2+976; 
			//rcValue[15] = ((sbus[21]>>5|sbus[22]<<3) & 0x07FF)/2+976; 
			
			// now the two Digital-Channels
			//if ((sbus[23]) & 0x0001)       rcValue[16] = 2000; else rcValue[16] = 1000;
			//if ((sbus[23] >> 1) & 0x0001)  rcValue[17] = 2000; else rcValue[17] = 1000;
			
			// Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
			#if defined(FAILSAFE)
			if (!((sbus[23] >> 3) & 0x0001))
			{if(failsafeCnt > 20) failsafeCnt -= 20; else failsafeCnt = 0;}   // clear FailSafe counter
			#endif
			
			// scaling for Futaba 8FG with NO EPA/travel limit
			for (int axis = XAXIS; axis < AUX3; axis++) {
				rcValue[axis] = map(rcValue[axis], 350, 1700, 1000, 2000);
			}
			rcChannel[XAXIS] = rcValue[0];
			rcChannel[YAXIS] = rcValue[1];
			rcChannel[ZAXIS] = rcValue[3];
			rcChannel[THROTTLE] = rcValue[2];
			rcChannel[MODE] = rcValue[4];
			rcChannel[AUX1] = rcValue[5];
			rcChannel[AUX2] = rcValue[6];
			rcChannel[AUX3] = rcValue[7];
		}
	}
}

// call readSBUS() only on first channel request
int getRawChannelValue(byte channel) {
	if (channel == XAXIS) {
		readSBUS();
	}
	return rcChannel[channel];
}

void setChannelValue(byte channel, int value) {
}

#endif

#endif