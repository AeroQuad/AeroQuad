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

SIGNAL(PCINT2_vect) {
  MegaPcIntISR();
}

//arduino pins 63, 64, 65, 62, 66, 67
static byte receiverPin[12] = {1, 2, 3, 0, 4, 5, 6, 7, 8, 9, 10, 11}; // bit number of PORTK used for XAXIS, YAXIS, ZAXIS, THROTTLE, MODE, AUX


void initializeReceiverPWM() {

  DDRK = 0;
  PORTK = 0;
  PCMSK2 |=(1<<nbReceiverChannel)-1;
  PCICR |= 0x1 << 2;

  for (byte channel = XAXIS; channel < nbReceiverChannel; channel++) {
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

// Channel data
volatile unsigned int startPulse = 0;
volatile byte         ppmCounter = MAX_NB_CHANNEL; // ignore data until first sync pulse
volatile int          PWM_RAW[MAX_NB_CHANNEL] = { 3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000 };

#define TIMER5_FREQUENCY_HZ 50
#define TIMER5_PRESCALER    8
#define TIMER5_PERIOD       (F_CPU/TIMER5_PRESCALER/TIMER5_FREQUENCY_HZ)

uint8_t rcChannel[] = {0,1,3,2,4,5,6,7,8,9,10,11,12};

/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER5_CAPT_vect)//interrupt.
{
  unsigned int stopPulse = ICR5;
  
  // Compensate for timer overflow if needed
  unsigned int pulseWidth = ((startPulse > stopPulse) ? TIMER5_PERIOD : 0) + stopPulse - startPulse;

  if (pulseWidth > 5000) {      // Verify if this is the sync pulse (2.5ms)
    ppmCounter = 0;             // -> restart the channel counter
  }
  else {
    if (ppmCounter < nbReceiverChannel) { // extra channels will get ignored here
      PWM_RAW[ppmCounter] = pulseWidth; // Store measured pulse length
      ppmCounter++;                     // Advance to next channel
    }
  }
  startPulse = stopPulse;         // Save time at pulse start
}



void initializeReceiverPPM() {

  pinMode(48, INPUT); // ICP5
  pinMode(A8, INPUT); // this is the original location of the first RX channel

  // Configure timer HW
  TCCR5A = ((1<<WGM50)|(1<<WGM51));
  TCCR5B = ((1<<WGM52)|(1<<WGM53)|(1<<CS51)|(1<<ICES5)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
  OCR5A = TIMER5_PERIOD; 

  TIMSK5 |= (1<<ICIE5); //Timer interrupt mask
}


int getRawChannelValuePPM(byte channel) {
  uint8_t oldSREG = SREG;
  cli(); // Disable interrupts to prevent race with ISR updating PWM_RAW

  int receiverRawValue = ((PWM_RAW[rcChannel[channel]])/2);

  SREG = oldSREG;
  
  return receiverRawValue;
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



