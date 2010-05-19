/*
  AeroQuad v1.8 - May 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
  
  Interrupt based method inspired by Dror Caspi
  http://www.rcgroups.com/forums/showpost.php?p=12356667&postcount=1639
  
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

#include "Receiver.h"

#ifdef Duemilanove_AQ1x
// Attaches PCINT to Arduino Pin
void attachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  uint8_t slot;
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }
  // set the mask
  *pcmask |= bit;
  // enable the interrupt
  PCICR |= 0x01 << port;
}

// Detaches PCINT from Arduino Pin
void detachPinChangeInterrupt(uint8_t pin) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *pcmask;

  // map pin to PCIR register
  if (port == NOT_A_PORT) {
    return;
  } 
  else {
    port -= 2;
    pcmask = port_to_pcmask[port];
  }

  // disable the mask.
  *pcmask &= ~bit;
  // if that's the last one, disable the interrupt.
  if (*pcmask == 0) {
    PCICR &= ~(0x01 << port);
  }
}

// ISR which records time of rising or falling edge of signal
static void measurePulseWidthISR(uint8_t port) {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = *portInputRegister(port+2);
  mask = curr ^ PCintLast[port];
  PCintLast[port] = curr;
  // mask is pins that have changed. screen out non pcint pins.
  if ((mask &= *port_to_pcmask[port]) == 0) {
    return;
  }
  currentTime = micros();
  // mask is pcint pins that have changed.
  for (uint8_t i=0; i < 8; i++) {
    bit = 0x01 << i;
    if (bit & mask) {
      pin = port * 8 + i;
      // for each pin changed, record time of change
      if (bit & PCintLast[port]) {
        time = currentTime - pinData[pin].fallTime;
        pinData[pin].riseTime = currentTime;        
        if ((time >= MINOFFWIDTH) && (time <= MAXOFFWIDTH))
          pinData[pin].edge = RISING_EDGE;
        else
          pinData[pin].edge == FALLING_EDGE; // invalid rising edge detected
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

SIGNAL(PCINT0_vect) {
  measurePulseWidthISR(0);
}
SIGNAL(PCINT1_vect) {
  measurePulseWidthISR(1);
}
SIGNAL(PCINT2_vect) {
  measurePulseWidthISR(2);
}
#endif

#ifdef Mega_AQ1x
void initializeMegaPcInt2() {
  DDRK = 0;
  PORTK = 0;
  PCMSK2 |= 0x3F;
  PCICR |= 0x1 << 2;
}

static void MegaPcIntISR() {
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint8_t pin;
  uint32_t currentTime;
  uint32_t time;

  //curr = PORTK;
  curr = *portInputRegister(11);
  mask = curr ^ PCintLast[0];
  PCintLast[0] = curr;  

  //Serial.println(curr,DEC);

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
          pinData[pin].edge == FALLING_EDGE; // invalid rising edge detected
      }
      else {
        time = currentTime - pinData[pin].riseTime;
        pinData[pin].fallTime = currentTime;
        if ((time >= MINONWIDTH) && (time <= MAXONWIDTH) && (pinData[pin].edge == RISING_EDGE)) {
          pinData[pin].lastGoodWidth = time;
          //Serial.println(pinData[4].lastGoodWidth);
          pinData[pin].edge = FALLING_EDGE;
        } 
      }
    }
  }
}

SIGNAL(PCINT2_vect) {
  MegaPcIntISR();
}
#endif

#ifdef AeroQuadAPM
/****************************************************
  Interrupt Vector
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
#endif

// Configure each receiver pin for PCINT
void configureReceiver() {
  #ifdef Duemilanove_AQ1x
  pinMode(THROTTLEPIN, INPUT);
  pinMode(ROLLPIN, INPUT);
  pinMode(PITCHPIN, INPUT);
  pinMode(YAWPIN, INPUT);
  pinMode(MODEPIN, INPUT);
  pinMode(AUXPIN, INPUT);
  for (channel = ROLL; channel < LASTCHANNEL; channel++) {
    attachPinChangeInterrupt(receiverChannel[channel]);
    pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  }
  #endif
  #ifdef Mega_AQ1x
  initializeMegaPcInt2();
  for (channel = ROLL; channel < LASTCHANNEL; channel++)
    pinData[receiverChannel[channel]].edge == FALLING_EDGE;
  #endif
  #ifdef AeroQuadAPM
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
  #endif
}

#ifndef AeroQuadAPM
// Calculate PWM pulse width of receiver data
// If invalid PWM measured, use last known good time
unsigned int readReceiver(byte receiverPin) {
  uint16_t data;
  uint8_t oldSREG;
    
  oldSREG = SREG;
  cli();
  data = pinData[receiverPin].lastGoodWidth;
  SREG = oldSREG;  
  return data;
}
#endif

#ifdef AeroQuadAPM
unsigned int readReceiver(byte receiverPin) {
  return (PWM_RAW[receiverPin]+600)/2;
}
#endif
