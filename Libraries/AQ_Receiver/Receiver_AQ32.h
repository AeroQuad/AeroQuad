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

#ifndef _AEROQUAD_RECEIVER_AQ32_H_
#define _AEROQUAD_RECEIVER_AQ32_H_

#include "Arduino.h"
#include "Receiver_Base_MEGA.h"

//#define STM32_TIMER_DEBUG // enable debug messages

///////////////////////////////////////////////////////////////////////////////
// configuration part starts here
// definition of pins used for PWM receiver input


/*
	ROLL     0	3
	PITCH    1	1
	YAW      2	0
	THROTTLE 3	2
	MODE     4	4
	AUX      5	6
	AUX2     6	5
	AUX3     7	7
*/

static byte receiverPin[] = {
    Port2Pin('D', 12),
    Port2Pin('D', 13),
    Port2Pin('D', 14),
    Port2Pin('D', 15),
    Port2Pin('E',  9),
    Port2Pin('E', 11),
    Port2Pin('E', 13),
    Port2Pin('E', 14)
  };

static byte ReceiverChannelMapPWM[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // default mapping


///////////////////////////////////////////////////////////////////////////////
// implementation part starts here.
// forward declaration, array is defined at the end of this file
extern voidFuncPtr PWM_in_handler[];

typedef struct {
  timer_dev   *TimerDev;
  timer_gen_reg_map *TimerRegs;
  __io uint32	*Timer_ccr;
  int			Low;
  int			High;
  uint16		HighTime;
  uint16		RiseTime;
  uint16		LastChange;
  int			Channel;
  int			TimerChannel;
  int			PolarityMask;
  int			Valid;
  int			Debug;
} tFrqDataPWM;

#define FRQInputs 8
volatile tFrqDataPWM FrqDataPWM[FRQInputs];

void FrqInit(int aChannel, int aDefault, volatile tFrqDataPWM *f, timer_dev *aTimer, int aTimerChannel) {

  aTimerChannel--;  // transform timer channel numbering from 1-4 to 0-3

  f->Channel      = aChannel;
  f->Valid        = false;

  f->TimerDev     = aTimer;
  timer_gen_reg_map *timer = aTimer->regs.gen;
  f->TimerRegs    = timer;

  f->Timer_ccr    = &timer->CCR1 + aTimerChannel;
  f->Debug        = false;
  f->HighTime     = aDefault;
  f->TimerChannel = aTimerChannel;

  int TimerEnable = (1 << (4*aTimerChannel));
  f->PolarityMask = TimerEnable << 1;

  uint32 clock_speed = rcc_dev_timer_clk_speed(f->TimerDev->clk_id);
  timer->PSC	= (clock_speed/1000000)-1;
  timer->ARR	= 0xffff;
  timer->CR1	= 0;
  timer->DIER &= ~(1);

  timer->CCER &= ~TimerEnable; // Disable timer
  timer->CCER &= ~(f->PolarityMask);

  volatile uint32 *mr;
  if(aTimerChannel < 2) {
    mr = &(timer->CCMR1);
  }
  else {
    mr = &(timer->CCMR2);
  }
  *mr &= ~(0xFF << (8*(aTimerChannel&1)));	// prescaler 1
  *mr |= 0x61 << (8*(aTimerChannel&1));		// 0x61 -> 6=filter, 1=inputs 1,2,3,4

  timer->CCER |= TimerEnable; // Enable
  timer->CR1 = 1;
}


void InitFrqMeasurementPWM() {

  for(int rcLine = 0; rcLine < (int)(sizeof(receiverPin) / sizeof(receiverPin[0])); rcLine++) {
    int pin = receiverPin[rcLine];
    timer_dev *timer_num = PIN_MAP[pin].timer_device;
    if(timer_num != NULL) {
      gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, GPIO_AF_INPUT_PD);
      FrqInit(rcLine, 1500, &FrqDataPWM[rcLine], timer_num, PIN_MAP[pin].timer_channel);
      timer_attach_interrupt(timer_num, PIN_MAP[pin].timer_channel, PWM_in_handler[rcLine]);
    }
  }
}


void PWMInvertPolarity(volatile tFrqDataPWM *f) {
  f->TimerRegs->CCER ^= f->PolarityMask; // invert polarity
}

void FrqChange(volatile tFrqDataPWM *f) {

  timer_gen_reg_map *timer = f->TimerRegs;
  uint16_t c = *(f->Timer_ccr);
  bool rising = (timer->CCER & f->PolarityMask) == 0;

  if(f->Valid) {
    if(rising) {
      f->RiseTime = c;
    } 
    else {
      uint16_t highTime = c - f->RiseTime;
      if(highTime > 900 && highTime < 2100) {
        f->HighTime = highTime;
      } 
      else {
        f->Valid = false;
      }
    }
  } 
  else if(rising) {
    // rising edge, store start time
    f->RiseTime = c;
    f->Valid = true;
  }

  PWMInvertPolarity(f);
}

// hide the class details from the interrupt handler
void IrqChangeValue(int chan) {
  FrqChange(&FrqDataPWM[chan]);
}


///////////////////////////////////////////////////////////////////////////////
// definition of interrupt handler functions, one for each channel
void PWM_in_0() { IrqChangeValue(0); }
void PWM_in_1() { IrqChangeValue(1); }
void PWM_in_2() { IrqChangeValue(2); }
void PWM_in_3() { IrqChangeValue(3); }
void PWM_in_4() { IrqChangeValue(4); }
void PWM_in_5() { IrqChangeValue(5); }
void PWM_in_6() { IrqChangeValue(6); }
void PWM_in_7() { IrqChangeValue(7); }

voidFuncPtr PWM_in_handler[] = { PWM_in_0, PWM_in_1, PWM_in_2, PWM_in_3, PWM_in_4, PWM_in_5, PWM_in_6, PWM_in_7 };


///////////////////////////////////////////////////////////////////////////////
// interface part starts here

void initializeReceiverPWM() {

	InitFrqMeasurementPWM();
}


int getRawChannelValuePWM(const byte channel) {
	int chan = ReceiverChannelMapPWM[channel];
	if(chan < (int)sizeof(receiverPin)) {
		volatile tFrqDataPWM *f = &FrqDataPWM[chan];
		uint16_t PulsLength = f->HighTime;
		return PulsLength;
	} else {
		return 1500;
	}
}




////////////////////////////////////////
// RECEIVER PPM for AQ r32
////////////////////////////////////////

static byte receiverPinPPM = Port2Pin('D', 15);

#define SERIAL_SUM_PPM               0,1,3,2,4,5,6,7,8,9,10,11 // ROLL,PITCH,THR,YAW... For Robe/Hitec/Futaba/Turnigy9xFrsky
static byte ReceiverChannelMapPPM[MAX_NB_CHANNEL] = {SERIAL_SUM_PPM};

uint16 rawChannelValue[MAX_NB_CHANNEL] =  {1500,1500,1500,1500,1500,1500,1500,1500,1500,1500};
byte   currentChannel;


///////////////////////////////////////////////////////////////////////////////
// implementation part starts here.

typedef struct {
  timer_dev   *TimerDev;
  timer_gen_reg_map *TimerRegs;
  __io uint32	*Timer_ccr;
  uint16		RiseTime;
  int			TimerChannel;
  int			PolarityMask;
} tFrqDataPPM;

volatile tFrqDataPPM FrqDataPPM;

void FrqInit(int aDefault, timer_dev *aTimer, int aTimerChannel)
{
  aTimerChannel--;  // transform timer channel numbering from 1-4 to 0-3

  FrqDataPPM.TimerDev     = aTimer;
  timer_gen_reg_map *timer = aTimer->regs.gen;
  FrqDataPPM.TimerRegs    = timer;

  FrqDataPPM.Timer_ccr    = &timer->CCR1 + aTimerChannel;
  FrqDataPPM.TimerChannel = aTimerChannel;

  int TimerEnable = (1 << (4*aTimerChannel));
  FrqDataPPM.PolarityMask = TimerEnable << 1;

  uint32 clock_speed = rcc_dev_timer_clk_speed(FrqDataPPM.TimerDev->clk_id);
  timer->PSC	= (clock_speed/1000000)-1;
  timer->ARR	= 0xffff;
  timer->CR1	= 0;
  timer->DIER &= ~(1);

  timer->CCER &= ~TimerEnable; // Disable timer
  timer->CCER &= ~(FrqDataPPM.PolarityMask);

  volatile uint32 *mr;
  if(aTimerChannel < 2) {
    mr = &(timer->CCMR1);
  } 
  else {
    mr = &(timer->CCMR2);
  }
  *mr &= ~(0xFF << (8*(aTimerChannel&1)));	// prescaler 1
  *mr |= 0x61 << (8*(aTimerChannel&1));		// 0x61 -> 6=filter, 1=inputs 1,2,3,4

  timer->CCER |= TimerEnable; // Enable
  timer->CR1 = 1;

}

void FrqChange()
{
  uint16_t c = *(FrqDataPPM.Timer_ccr);
  uint16_t diffTime = c - FrqDataPPM.RiseTime;
  if ((diffTime > 900) && (diffTime < 2100)) {
    if (currentChannel < MAX_NB_CHANNEL) {
      rawChannelValue[currentChannel] = diffTime;
      currentChannel++;
    }
  } 
  else if (diffTime > 2500) {
    currentChannel = 0;
  } 
  else {
    // glitch; stop and wait next round
    currentChannel = MAX_NB_CHANNEL;
  }
  FrqDataPPM.RiseTime = c;
}

void InitFrqMeasurementPPM()
{
  int pin = receiverPinPPM;
  timer_dev *timer_num = PIN_MAP[pin].timer_device;

  currentChannel=8;
  if(timer_num != NULL) {
    gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, GPIO_AF_INPUT_PD);
    FrqInit(1500, timer_num, PIN_MAP[pin].timer_channel);
    timer_attach_interrupt(timer_num, PIN_MAP[pin].timer_channel, FrqChange);
  }

}



///////////////////////////////////////////////////////////////////////////////
// interface part starts here

void initializeReceiverPPM() {
  
  InitFrqMeasurementPPM();
}


int getRawChannelValuePPM(const byte channel) {
  return rawChannelValue[ReceiverChannelMapPPM[channel]];
}


#endif



