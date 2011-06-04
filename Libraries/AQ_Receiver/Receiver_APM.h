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

#include <WProgram.h>
#include "Receiver.h"

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include "Receiver_APM.h"
#include <AQMath.h>
#include <Axis.h>

#include <avr/interrupt.h>
volatile unsigned int Start_Pulse = 0;
volatile unsigned int Stop_Pulse = 0;
volatile unsigned int Pulse_Width = 0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {
  2400,2400,2400,2400,2400,2400,2400,2400};

/****************************************************
 * Interrupt Vector
 ****************************************************/
ISR(TIMER4_CAPT_vect)//interrupt.
{
  if(((1<<ICES4)&TCCR4B) >= 0x01)
  {

    if(Start_Pulse>Stop_Pulse) //Checking if the Stop Pulse overflow the register, if yes i normalize it.
    {
      Stop_Pulse+=40000; //Nomarlizing the stop pulse.
    }
    Pulse_Width=Stop_Pulse-Start_Pulse; //Calculating pulse
    if(Pulse_Width>5000) //Verify if this is the sync pulse
    {
      PPM_Counter=0; //If yes restart the counter
    }
    else
    {
      //PWM_RAW[PPM_Counter]=Pulse_Width; //Saving pulse.
      PWM_RAW[PPM_Counter & 0x07]=Pulse_Width; //Saving pulse.
      PPM_Counter++;
    }
    Start_Pulse=ICR4;
    TCCR4B &=(~(1<<ICES4)); //Changing edge detector.
  }
  else
  {
    Stop_Pulse=ICR4; //Capturing time stop of the drop edge
    TCCR4B |=(1<<ICES4); //Changing edge detector.
    //TCCR4B &=(~(1<<ICES4));
  }
  //Counter++;
}

class Receiver_APM : public Receiver {
private:
  int receiverPin[6];
  
public:  
  Receiver_APM() {

    receiverPin[ROLL] = 0;
    receiverPin[PITCH] = 1;
    receiverPin[YAW] = 3;
    receiverPin[THROTTLE] = 2;
    receiverPin[MODE] = 4;
    receiverPin[AUX] = 5;
  }

  void initialize(void) {
    /*Note that timer4 is configured to used the Input capture for PPM decoding and to pulse two servos
     OCR4A is used as the top counter*/
    pinMode(49, INPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);
    //Remember the registers not declared here remains zero by default...
    TCCR4A =((1<<WGM40)|(1<<WGM41)|(1<<COM4C1)|(1<<COM4B1)|(1<<COM4A1));
    TCCR4B = ((1<<WGM43)|(1<<WGM42)|(1<<CS41)|(1<<ICES4)); //Prescaler set to 8, that give us a resolution of 2us, read page 134 of data sheet
    OCR4A = 40000; ///50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
    //must be 50hz because is the servo standard (every 20 ms, and 1hz = 1sec) 1000ms/20ms=50hz, elementary school stuff...
    OCR4B = 3000; //PH4, OUT5
    OCR4C = 3000; //PH5, OUT4

    TIMSK4 |= (1<<ICIE4); //Timer interrupt mask
    sei();
  }

  void read(void) {
    for(byte channel = ROLL; channel < LASTCHANNEL; channel++) {
      //currentTime = micros();
      // Apply transmitter calibration adjustment
      receiverData[channel] = (mTransmitter[channel] * ((PWM_RAW[receiverPin[channel]]+600)/2)) + bTransmitter[channel];
      // Smooth the flight control transmitter inputs
      transmitterCommandSmooth[channel] = filterSmooth(receiverData[channel], transmitterCommandSmooth[channel], transmitterSmooth[channel]);
      //previousTime = currentTime;
    }

    // Reduce transmitter commands using xmitFactor and center around 1500
    for (byte channel = ROLL; channel < THROTTLE; channel++)
      transmitterCommand[channel] = ((transmitterCommandSmooth[channel] - transmitterZero[channel]) * xmitFactor) + transmitterZero[channel];
    // No xmitFactor reduction applied for throttle, mode and
    for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
      transmitterCommand[channel] = transmitterCommandSmooth[channel];
  }
};
#endif

#endif


