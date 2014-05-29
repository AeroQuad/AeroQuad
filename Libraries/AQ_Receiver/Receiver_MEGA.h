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

#ifndef _AEROQUAD_RECEIVER_MEGA_H_
#define _AEROQUAD_RECEIVER_MEGA_H_

#include "Arduino.h"
#include "Receiver_Base_MEGA.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

#include "pins_arduino.h"
#include <AQMath.h>
#include "GlobalDefined.h"


volatile uint8_t *port_to_pcmask[] = {
  &PCMSK0,
  &PCMSK1,
  &PCMSK2
};
volatile static uint8_t PCintLast[3];
// Channel data
typedef struct {
  byte edge;
  unsigned long riseTime;
  unsigned long fallTime;
  unsigned int lastGoodWidth;
} tPinTimingData;
volatile static tPinTimingData pinData[9];

static void MegaPcIntISR() {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;

  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= PCMSK2) == 0) {
    return;
  }

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = i;
      // for each pin changed, record time of change
      if (bit & PCintLast[0]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge = FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          pinData[pin].edge = FALLING_EDGE;
        }
      }
    }
  }
}




//arduino pins 63, 64, 65, 62, 66, 67
static byte receiverPin[12] = {1, 2, 3, 0, 4, 5, 6, 7}; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX


void initializeReceiverPWM() {

  DDRK = 0;
  PORTK = 0;
  PCMSK2 |=(1<<LAST_CHANNEL)-1;
  PCICR |= 0x1 << 2;

  for (byte channel = XAXIS; channel < LAST_CHANNEL; channel++) {
    pinData[receiverPin[channel]].edge = FALLING_EDGE;
  }
}

int getRawChannelValuePWM(byte channel) {
  byte pin = receiverPin[channel];
  uint8_t oldSREG = SREG;
  cli();
  // Get receiver value read by pin change interrupt handler
  uint16_t receiverRawValue = pinData[pin].lastGoodWidth;
  SREG = oldSREG;
  
  return receiverRawValue;
}



//////////////////////////////////////////////
// PPM receiver function definition
//////////////////////////////////////////////


static uint8_t rcChannel[MAX_NB_CHANNEL] = {0,1,2,3,4,5,6,7};
volatile uint16_t rcValue[MAX_NB_CHANNEL] = {1500,1500,1500,1500,1500,1500,1500,1500}; // interval [1000;2000]

#if defined (PPM_ON_THROTTLE)
	#define PPM_PIN_INTERRUPT() DDRK &= ~(1<<0); PORTK |= (1<<0);  PCICR |= (1<<2); PCMSK2 |= (1<<0);
#else
	#define PPM_PIN_INTERRUPT() attachInterrupt(4, rxInt, RISING) //PIN 19, also used for Spektrum satellite option
#endif

static void rxInt() {
  uint16_t now,diff;
  static uint16_t last = 0;
  static uint8_t chan = MAX_NB_CHANNEL;

  now = micros();
  diff = now - last;
  last = now;
  if(diff>3000) { 
    chan = 0;
  }
  else if( 800 < diff && diff < 2200 && chan < MAX_NB_CHANNEL ) {
    rcValue[chan] = diff;
    chan++;
  }
  else {
    chan = MAX_NB_CHANNEL;
  }
}


SIGNAL(PCINT2_vect) {
  if (receiverTypeUsed == RECEIVER_PWM) {
	MegaPcIntISR();
  }
  #if defined (PPM_ON_THROTTLE)
	if (receiverTypeUsed == RECEIVER_PPM) {
		if(PINK & (1<<0)) {
			rxInt();
		}
	}
  #endif
}

void initializeReceiverPPM() {

  PPM_PIN_INTERRUPT(); 
}


int getRawChannelValuePPM(byte channel) {

  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts

  int rawChannelValue = rcValue[rcChannel[channel]];
  SREG = oldSREG;
  
  return rawChannelValue;
}


//////////////////////////////////////////////
// SBUS receiver function definition
//////////////////////////////////////////////

#define SBUS_SYNCBYTE 0x0F // some sites say 0xF0
#define SERIAL_SBUS Serial3

int rawChannelValue[MAX_NB_CHANNEL] =  {1500,1500,1500,1500,1500,1500,1500};
static byte ReceiverChannelMapSBUS[MAX_NB_CHANNEL] = {0,1,2,3,4,5,6,7};
static unsigned int sbusIndex = 0;
// sbus rssi reader variables
static unsigned short sbusFailSafeCount = 0;
static unsigned long sbusFrameCount = 0;
static unsigned short sbusRate = 0;
boolean useSbusRSSIReader = false;




///////////////////////////////////////////////////////////////////////////////
// implementation part starts here.

#define SBUS_ENDBYTE 0x00

static unsigned int sbusPacketLength = 24;

void readSBUS()
{
    static byte sbus[25] = {0};
    while(SERIAL_SBUS.available()) {
        if(SERIAL_SBUS.available() > 124) { //input buffer is backed up and may cause lag - dump it to get back on track - set to 5 packets
            while(SERIAL_SBUS.available()) SERIAL_SBUS.read();//dump the input buffer
            sbusIndex = 0; //clear the packet buffer
        }
        else 
        {
            int val = SERIAL_SBUS.read();
            if(sbusIndex == 0 && val != SBUS_SYNCBYTE) {
                continue;
            }
            
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
                    rawChannelValue[XAXIS]      = ((sbus[1]     | sbus[2]<<8)  & 0x07FF);                    // pitch
                    rawChannelValue[YAXIS]      = ((sbus[2]>>3  | sbus[3]<<5)  & 0x07FF);                    // roll
                    rawChannelValue[THROTTLE]   = ((sbus[3]>>6  | sbus[4]<<2   | sbus[5]<<10) & 0x07FF);    // throttle
                    rawChannelValue[ZAXIS]      = ((sbus[5]>>1  | sbus[6]<<7)  & 0x07FF);                    // yaw
                    rawChannelValue[MODE]       = ((sbus[6]>>4  | sbus[7]<<4)  & 0x07FF);
                    rawChannelValue[AUX1]       = ((sbus[7]>>7  | sbus[8]<<1   | sbus[9]<<9) & 0x07FF);
                    rawChannelValue[AUX2]       = ((sbus[9]>>2  | sbus[10]<<6) & 0x07FF);
                    rawChannelValue[AUX3]       = ((sbus[10]>>5 | sbus[11]<<3) & 0x07FF);
                    rawChannelValue[AUX4]       = ((sbus[12]    | sbus[13]<<8) & 0x07FF);
                    rawChannelValue[AUX5]       = ((sbus[13]>>3 | sbus[14]<<5) & 0x07FF);
                    //rawChannelValue[AUX6]        = ((sbus[14]>>6 | sbus[15]<<2|sbus[16]<<10) & 0x07FF);
                    //rawChannelValue[AUX7]        = ((sbus[16]>>1 | sbus[17]<<7) & 0x07FF);
                    
                    
                    if (useSbusRSSIReader) {
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
                    }    
                
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

///////////////////////////////////////////////////////////////////////////////
// interface part starts here

void initializeReceiverSBUS() {
    SERIAL_SBUS.begin(100000);
}

// use this to switch from one receiver type to another?
// re-enables serial port for other uses
//void terminateReceiverSBUS() {
    //SERIAL_SBUS.end();
//}

int getRawChannelValueSBUS(const byte channel) {
    if (channel == XAXIS) {
        readSBUS();
	}
	return rawChannelValue[ReceiverChannelMapSBUS[channel]];
}


#endif



