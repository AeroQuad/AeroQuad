/*
  AeroQuad v2.2 - Feburary 2011
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

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#include "ReceiverForAPM.h"
#include <AQMath.h>

#include <avr/interrupt.h>
volatile unsigned int Start_Pulse = 0;
volatile unsigned int Stop_Pulse = 0;
volatile unsigned int Pulse_Width = 0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {2400,2400,2400,2400,2400,2400,2400,2400};

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
      PWM_RAW[PPM_Counter]=Pulse_Width; //Saving pulse.
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
//#endif

ReceiverForAPM::ReceiverForAPM()
{
  receiverPin[ROLL] = 0;
  receiverPin[PITCH] = 1;
  receiverPin[YAW] = 3;
  receiverPin[THROTTLE] = 2;
  receiverPin[MODE] = 4;
  receiverPin[AUX] = 5;
}

void ReceiverForAPM::initialize() 
{
  this->_initialize(); // load in calibration and xmitFactor from EEPROM
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

void ReceiverForAPM::read() 
{
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
  {
    _currentTime = micros();
    // Apply transmitter calibration adjustment
    _receiverData[channel] = (_mTransmitter[channel] * ((PWM_RAW[receiverPin[channel]]+600)/2)) + _bTransmitter[channel];
    // Smooth the flight control transmitter inputs
    _transmitterCommandSmooth[channel] = filterSmooth(_receiverData[channel], _transmitterCommandSmooth[channel], _transmitterSmooth[channel]);
    _previousTime = _currentTime;
  }

  // Reduce transmitter commands using xmitFactor and center around 1500
  for (byte channel = ROLL; channel < THROTTLE; channel++)
  {
    _transmitterCommand[channel] = ((_transmitterCommandSmooth[channel] - _transmitterZero[channel]) * _xmitFactor) + _transmitterZero[channel];
  }
  // No xmitFactor reduction applied for throttle, mode and
  for (byte channel = THROTTLE; channel < LASTCHANNEL; channel++)
  {
    _transmitterCommand[channel] = _transmitterCommandSmooth[channel];
  }
}

#endif
