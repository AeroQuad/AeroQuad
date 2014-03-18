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
#define SBUS_ENDBYTE 0x00
  
#define SERIAL_SBUS Serial2  

// 16 analog, 2 digital channels
static unsigned int rcChannel[18] = {XAXIS,YAXIS,THROTTLE,ZAXIS,MODE,AUX1,AUX2,AUX3,AUX4,AUX5,10,11,12,13,14,15,16,17};
static unsigned int sbusIndex = 0;
static unsigned int sbusPacketLength = 24;  // index length modify as required
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
		if(SERIAL_SBUS.available() > 124) { //input buffer is backed up and may cause lag - dump it to get back on track - set to 5 packets
			while(SERIAL_SBUS.available()) SERIAL_SBUS.read();//dump the input buffer
			sbusIndex = 0; //clear the packet buffer
		}
		else 
		{
			int val = SERIAL_SBUS.read();
			if(sbusIndex == 0 && val != SBUS_SYNCBYTE) continue; //search the input buffer for the beginning of a packet
			
			sbus[sbusIndex] = val;
			
			if (sbusIndex == sbusPacketLength) { //we have a full packet
				if (val != SBUS_ENDBYTE) { //out of sync incorrect end byte
					int shiftIndex = 0;
					for(int i=1;i<=sbusIndex;i++){ // start at array pos 2 because we already know the byte at pos 1 is a syncByte
						if(sbus[i] == SBUS_SYNCBYTE){
							shiftIndex = i;
							break; //we have the location of the next SYNCBYTE
						}
					}

					if(shiftIndex != 0) { //the start of a packet was found in the middle of the bad packet
						//shift everything by the value of -shiftIndex
						for(int i=0;i<=sbusPacketLength-shiftIndex;i++){
							sbus[i] = sbus[i+shiftIndex];
						}

						//reset the sbusIndex to the next location
						sbusIndex = sbusIndex - shiftIndex;
						sbusIndex++;
					}
					else { //no packet start was found in the middle of the bad packet
						sbusIndex = 0; //clear the packet buffer
					}
				} 
				else 
				{ //everything is OK as my end byte and sync byte are correct			  
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
				   
				   sbusIndex = 0; //clear the packet buffer				   
				}			 
			}			
			else 
			{ //we have a partial packet - keep calm and carry on
				sbusIndex++;
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