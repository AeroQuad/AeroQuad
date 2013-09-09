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

#include <AQMath.h>
#include "GlobalDefined.h"

volatile uint16_t PPMt[16]; // unvalidated input
volatile uint16_t PPM[16];
volatile uint8_t  PPMch = 255;
volatile uint32_t PPMlast=0;
volatile uint32_t pwmLast[8];


void pwmHandler(uint8_t ch, uint32_t pin) {
  uint32_t cv = TC0->TC_CHANNEL[1].TC_CV;
  if (digitalRead(pin)) {
    pwmLast[ch] = cv;
  } else {
    cv = (cv - pwmLast[ch]) / 42;
    if (cv>750 && cv<2250) {
      PPM[ch] = cv;
    }
  }
}

void ch1Handler() { pwmHandler(0, 44); }
void ch2Handler() { pwmHandler(1, 45); }
void ch3Handler() { pwmHandler(2, 46); }
void ch4Handler() { pwmHandler(3, 47); }
void ch5Handler() { pwmHandler(4, 48); }
void ch6Handler() { pwmHandler(5, 49); }
void ch7Handler() { pwmHandler(6, 50); }
void ch8Handler() { pwmHandler(7, 51); }


void initializeReceiverPWM() {
  pmc_enable_periph_clk(ID_TC1);
  // use timer just for timing reference
  TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Start(TC0,1);
  attachInterrupt(44,ch1Handler,CHANGE);
  attachInterrupt(45,ch2Handler,CHANGE);
  attachInterrupt(46,ch3Handler,CHANGE);
  attachInterrupt(47,ch4Handler,CHANGE);
  attachInterrupt(48,ch5Handler,CHANGE);
  attachInterrupt(49,ch6Handler,CHANGE);
  attachInterrupt(50,ch7Handler,CHANGE);
  attachInterrupt(51,ch8Handler,CHANGE);
}

int getRawChannelValuePWM(byte channel) {  
  return PPM[channel];
}

int getRawChannelValuePPM(byte channel) {  
  return PPM[channel];
}

void TC1_Handler() {
  if (TC0->TC_CHANNEL[1].TC_SR & TC_SR_LDRAS) {
    uint32_t out, now = TC0->TC_CHANNEL[1].TC_RA;
    out = (now - PPMlast) / 42;
    PPMlast = now;
    if ((out >= 750) && (out < 2250)) {
      // valid pulse...
      if (PPMch < 16) {
        PPMt[PPMch++] = out;
      }
    } else if (out > 2500) {
      // valid sync pulse
      if (PPMch <=16) {
        for (uint8_t i = 0; i < PPMch; i++) {
          PPM[i] = PPMt[i];
        }
      } 
      PPMch=0;
    } else {
      // glitch
      PPMch=255;
    }
  }
}

void setTCpin(uint32_t pin) {
  PIO_Configure(g_APinDescription[pin].pPort,
                g_APinDescription[pin].ulPinType,
                g_APinDescription[pin].ulPin,
                g_APinDescription[pin].ulPinConfiguration);
}

void initializeReceiverPPM() {
  pmc_enable_periph_clk(ID_TC1);
  TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_LDRA_RISING | TC_CMR_LDRB_FALLING);
  setTCpin(A7);
  NVIC_EnableIRQ(TC1_IRQn);
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_LDRAS;
  TC_Start(TC0,1);
}
//////////////////////////////////////////////
// SBUS receiver function definition
//////////////////////////////////////////////

#define SBUS_SYNCBYTE 0x0F // some sites say 0xF0
#define SERIAL_SBUS Serial3

int rawChannelValue[MAX_NB_CHANNEL] =  {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};
static byte ReceiverChannelMapSBUS[MAX_NB_CHANNEL] = {0,1,2,3,4,5,6,7,8,9,10,11};
static unsigned int sbusIndex = 0;
// sbus rssi reader variables
static unsigned short sbusFailSafeCount = 0;
static unsigned long sbusFrameCount = 0;
static unsigned short sbusRate = 0;
boolean useSbusRSSIReader = false;




///////////////////////////////////////////////////////////////////////////////
// implementation part starts here.

void readSBUS()
{
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
                
                rawChannelValue[XAXIS]      = ((sbus[1]     | sbus[2]<<8)  & 0x07FF);					// pitch
                rawChannelValue[YAXIS]      = ((sbus[2]>>3  | sbus[3]<<5)  & 0x07FF);					// roll
                rawChannelValue[THROTTLE]   = ((sbus[3]>>6  | sbus[4]<<2   | sbus[5]<<10) & 0x07FF);	// throttle
                rawChannelValue[ZAXIS]      = ((sbus[5]>>1  | sbus[6]<<7)  & 0x07FF);					// yaw
                rawChannelValue[MODE]       = ((sbus[6]>>4  | sbus[7]<<4)  & 0x07FF);
                rawChannelValue[AUX1]       = ((sbus[7]>>7  | sbus[8]<<1   | sbus[9]<<9) & 0x07FF);
                rawChannelValue[AUX2]       = ((sbus[9]>>2  | sbus[10]<<6) & 0x07FF);
                rawChannelValue[AUX3]       = ((sbus[10]>>5 | sbus[11]<<3) & 0x07FF);
                rawChannelValue[AUX4]       = ((sbus[12]    | sbus[13]<<8) & 0x07FF);
                rawChannelValue[AUX5]       = ((sbus[13]>>3 | sbus[14]<<5) & 0x07FF);
                //rawChannelValue[AUX6]		= ((sbus[14]>>6 | sbus[15]<<2|sbus[16]<<10) & 0x07FF);
                //rawChannelValue[AUX7]		= ((sbus[16]>>1 | sbus[17]<<7) & 0x07FF);
                
                
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



