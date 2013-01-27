/*
  Copyright (c) 2011 ala42.  All rights reserved.

  STM32 receiver class by ala42 using time input capture
  for use with AeroQuad software and Maple library
  V 1.0 Oct 15 2011
  V 1.1 Jan 22 2012	class free version for AeroQuad 3.0 compatibility

  Define the pin numbers used for the receiver in receiverPin[]

  Timer and timer channels are accessed using the Maple PIN_MAP array.
  Make sure libmaple and this receiver class are compiled using the
  same structure alignment mode. When in doubt, change the stm32_pin_info
  declaration in wirish_types.h to align the size to a multiple of 4 byte
  by adding a filler byte at the end of the structure declaration.
*/

#ifndef _AEROQUAD_RECEIVER_STM32_H_
#define _AEROQUAD_RECEIVER_STM32_H_

#if defined(AeroQuadSTM32)

#include "Receiver.h"
#include "wirish.h"

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
static byte ReceiverChannelMap[] = {0, 1, 2, 3, 4, 5, 6, 7}; // default mapping


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
} tFrqData;

#define FRQInputs 8
volatile tFrqData FrqData[FRQInputs];

void FrqInit(int aChannel, int aDefault, volatile tFrqData *f, timer_dev *aTimer, int aTimerChannel) {

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


void InitFrqMeasurement() {

  for(int rcLine = 0; rcLine < (int)(sizeof(receiverPin) / sizeof(receiverPin[0])); rcLine++) {
    int pin = receiverPin[rcLine];
    timer_dev *timer_num = PIN_MAP[pin].timer_device;
    if(timer_num != NULL) {
      gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, GPIO_AF_INPUT_PD);
      FrqInit(rcLine, 1500, &FrqData[rcLine], timer_num, PIN_MAP[pin].timer_channel);
      timer_attach_interrupt(timer_num, PIN_MAP[pin].timer_channel, PWM_in_handler[rcLine]);
    }
  }
}


void PWMInvertPolarity(volatile tFrqData *f) {
  f->TimerRegs->CCER ^= f->PolarityMask; // invert polarity
}

void FrqChange(volatile tFrqData *f) {

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
  FrqChange(&FrqData[chan]);
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

void initializeReceiver(int nbChannel = 8) {

    initializeReceiverParam(nbChannel);
	InitFrqMeasurement();
}


int getRawChannelValue(const byte channel) {
	int chan = ReceiverChannelMap[channel];
	if(chan < (int)sizeof(receiverPin)) {
		volatile tFrqData *f = &FrqData[chan];
		uint16_t PulsLength = f->HighTime;
		return PulsLength;
	} else {
		return 1500;
	}
}


void setChannelValue(byte channel,int value) {
}

#endif

#endif
