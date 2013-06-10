/*
  STM32 SoftModem for AQ32 by kha

  Copyright (c) 2012 AeroQuad developers.  All rights reserved.

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

  Uses 
  - DAC channel 1 to pin PA4
  - Timers 6 and 7
  - DMA1 Stream 5
  
*/

#ifndef _AEROQUAD_SOFTMODEM_H_
#define _AEROQUAD_SOFTMODEM_H_

#if !defined(AeroQuadSTM32)
  #error SoftModem is only supported on STM32
#endif

#include <wirish/wirish.h>
#include <dac.h>
#include <dma.h>


#ifdef SOFTMODEM_FSKv2
  // TCMS105 clocked at 8Mhz
  #define ARR_MARK 1118 // 2345 Hz
  #define ARR_SPACE 692 // 3789 Hz
#else
  // Standard 1300/2100
  #define ARR_MARK 2016  // 1300 Hz
  #define ARR_SPACE 1249 // 2100 Hz
#endif

#ifndef SOFTMODEM_BAUDRATE
  #define BAUDRATE 1200
#endif


#define SOFTMODEM_PIN    Port2Pin('A',4)

unsigned char softmodemDMABuffer[32] = { // sine wave 32 samples
  128,153,177,199,219,234,246,253,255,253,246,234,219,199,177,153,
  128,103, 79, 57, 37, 22, 10,  2,  0,  2, 10, 22, 37, 57, 79,103};

volatile byte softmodemCurrentBit=0; // 1 - startbit, 2-9 databits, 10 stop, 11 stop2
volatile byte softmodemCurrentByte='A';

void softmodemInterrupt() {
  TIMER7_BASE->SR&=~1; // gludge... may get double int if this is not done
  if (softmodemCurrentBit) {
    if (softmodemCurrentBit == 1) {
      TIMER6_BASE->ARR = ARR_SPACE; // START bit
    }
    else if (softmodemCurrentBit == 10) {
      TIMER6_BASE->ARR = ARR_MARK;
    }
    else if (softmodemCurrentBit > 10) {
      softmodemCurrentBit = 0;
      return; // do not increase counter 
    }
    else {
      if (softmodemCurrentByte & (1 << (softmodemCurrentBit - 2))) {
	TIMER6_BASE->ARR = ARR_MARK;
      }  else {
	TIMER6_BASE->ARR = ARR_SPACE;
      }
    }
    softmodemCurrentBit++;
  }
}

void softmodemInit()
{
  dac_init(DAC,0); // do not enable yet
  pinMode(SOFTMODEM_PIN, INPUT_ANALOG);

  dma_init(DMA1);
  dma_setup_transfer(DMA1, DMA_STREAM5, &DAC->regs->DHR8R1, softmodemDMABuffer, softmodemDMABuffer,
		     DMA_CR_CH7|DMA_CR_CT1|DMA_CR_DBM|DMA_CR_PL_VERY_HIGH|DMA_CR_MINC|DMA_CR_CIRC|DMA_CR_DIR_M2P,
		     0);
  //		   0x0e0f0540,1);

  dma_set_num_transfers(DMA1,DMA_STREAM5,32);

  DAC->regs->DHR8R1 = 0x7f;
  DAC->regs->CR &= 0xffff0000;     // clear bits for ch1
  DAC->regs->CR |= DAC_CR_DMAEN1;  // enable DMA
  DAC->regs->CR |= DAC_CR_TEN1;    // enable trigger (timer6 TRGO)
  dac_enable_channel(DAC,DAC_CH1);

  dma_enable(DMA1,DMA_STREAM5);

  timer_init(TIMER6);
  TIMER6_BASE->ARR = ARR_MARK;
  TIMER6_BASE->PSC = 0;
  TIMER6_BASE->CR2 = TIMER_CR2_MMS_UPDATE;
  TIMER6_BASE->CR1 = TIMER_CR1_ARPE | TIMER_CR1_URS | TIMER_CR1_CEN;

  timer_init(TIMER7);
  timer_attach_interrupt(TIMER7,TIMER_UPDATE_INTERRUPT,softmodemInterrupt);
  nvic_irq_set_priority(NVIC_TIMER7,1);

  TIMER7_BASE->ARR = 1288800/BAUDRATE;
  TIMER7_BASE->PSC = 63;
  TIMER7_BASE->CR2 = TIMER_CR2_MMS_ENABLE;
  TIMER7_BASE->CR1 = TIMER_CR1_URS | TIMER_CR1_CEN;
}


byte softmodemFreeToSend() {
  return (softmodemCurrentBit==0);
}

void softmodemSendByte(unsigned char data) {
  if (!softmodemCurrentBit) { 
    softmodemCurrentByte = data;
    softmodemCurrentBit=1;
  }
}

#endif
