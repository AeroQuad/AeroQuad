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

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined (AeroQuadSTM32)

#if !defined (AeroQuadSTM32)
  #include "Arduino.h"
  #include "pins_arduino.h"
  #include <AQMath.h>
  #include "GlobalDefined.h"
#endif
#include "Receiver.h"

#define SBUS_SYNCBYTE 0x0F // some sites say 0xF0
  
#define SERIAL_SBUS Serial3  

// 16 analog, 2 digital channels
static unsigned int rcChannel[18] = {XAXIS,YAXIS,THROTTLE,ZAXIS,MODE,AUX1,AUX2,AUX3,AUX4,AUX5,10,11,12,13,14,15,16,17};
static unsigned int sbusIndex = 0;
#if defined (UseSBUSRSSIReader)
  static unsigned short sbusFailSafeCount = 0;
  static unsigned long sbusFrameCount = 0;
  static unsigned short sbusRate = 0;
#endif

void initializeReceiver(int nbChannel = 10) {
  initializeReceiverParam(nbChannel);
  #if defined (AeroQuadSTM32)
    pinMode(BOARD_SPI2_NSS_PIN, OUTPUT);
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH); // GPIO PB12 /Libmaple/libmaple/wirish/boards/aeroquad32.h line 69
  #endif
  SERIAL_SBUS.begin(100000);
}

void readSBUS() {

  static byte sbus[25] = {0};
  while(SERIAL_SBUS.available()) {
  
    int val = SERIAL_SBUS.read();
    if(sbusIndex == 0 && val != SBUS_SYNCBYTE) {
      continue;
    }
	
    sbus[sbusIndex] = val;
    sbusIndex++;
    if (sbusIndex == 25) {
	
      sbusIndex = 0;
      // check stop bit before updating buffers
      if (sbus[24] == 0x0) {
	  
        rcChannel[XAXIS]	= ((sbus[1]     | sbus[2]<<8)  & 0x07FF);					// pitch
        rcChannel[YAXIS]	= ((sbus[2]>>3  | sbus[3]<<5)  & 0x07FF);					// roll
        rcChannel[THROTTLE] = ((sbus[3]>>6  | sbus[4]<<2   | sbus[5]<<10) & 0x07FF);	// throttle
        rcChannel[ZAXIS]	= ((sbus[5]>>1  | sbus[6]<<7)  & 0x07FF);					// yaw
        rcChannel[MODE]		= ((sbus[6]>>4  | sbus[7]<<4)  & 0x07FF);
        rcChannel[AUX1]		= ((sbus[7]>>7  | sbus[8]<<1   | sbus[9]<<9) & 0x07FF);
        rcChannel[AUX2]		= ((sbus[9]>>2  | sbus[10]<<6) & 0x07FF);
        rcChannel[AUX3]		= ((sbus[10]>>5 | sbus[11]<<3) & 0x07FF);
        rcChannel[AUX4]		= ((sbus[12]    | sbus[13]<<8) & 0x07FF);
        rcChannel[AUX5]		= ((sbus[13]>>3 | sbus[14]<<5) & 0x07FF);
        //rcChannel[AUX6]		= ((sbus[14]>>6 | sbus[15]<<2|sbus[16]<<10) & 0x07FF);
        //rcChannel[AUX7]		= ((sbus[16]>>1 | sbus[17]<<7) & 0x07FF);


	   #ifdef UseSBUSRSSIReader
		if (sbusRate == 0) {
		  sbusFrameCount++;
		}
		if (((sbus[23] >> 3) & 0x0001)) {
		  if ((sbusRate > 0) && (sbusFailSafeCount < sbusRate)) {
		    sbusFailSafeCount++;
		  }
		} else if (sbusFailSafeCount > 0) {
		  sbusFailSafeCount--;
		}
	   #endif
      }
    }
  }
}

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