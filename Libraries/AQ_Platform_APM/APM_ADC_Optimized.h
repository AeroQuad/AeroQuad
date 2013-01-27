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

#ifndef _APM_ADC_OPTIMIZED_H_
#define _APM_ADC_OPTIMIZED_H_

#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)


#include <inttypes.h>
#include <avr/interrupt.h>

//#define SPI_MEASURE
#define ADC_SPI_LASTCHANNEL 6

#define bit_set(p,m)   ((p) |=  (1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))

// We use Serial Port 2 in SPI Mode
#define ADC_DATAOUT     51    // MOSI
#define ADC_DATAIN      50    // MISO
#define ADC_SPICLOCK    52    // SCK
#define ADC_CHIP_SELECT 33    // PC4   9 // PH6  Port: 0x08, Bit mask: 0x40

// Commands for reading ADC channels on ADS7844
//                                 pRate  qRate  rRate  aX     aY     aZ     temp   JP5
// ADC Input Channel               Ch1    Ch2    Ch0    Ch4    Ch5    Ch6    Ch3    Ch7
const unsigned char adc_cmd[9] = { 0xC7,  0x97,  0x87,  0xA7,  0xE7,  0xB7,  0xD7,  0xF7,  0x00 };

// Commands for reading ADC channels on ADS7844  (old AQ way
// ADC channel mapping             Ch0   Ch1   Ch2   Ch3   Ch4   Ch5   Ch6   Ch7
//const unsigned char adc_cmd[9]=  { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };


typedef struct {
	long value;
	unsigned char numberOfSamples;
	unsigned char cmd;
} tADCData;

tADCData ADCData[8];


void ADC_SPI_WaitTransmitBufferEmpty() {
  while ( !( UCSR2A & (1<<UDRE2)) )
    ;
}

void ADC_SPI_WaitReceiveBufferFull() {
  while ( !(UCSR2A & (1<<RXC2)) )
    ;
}

void ADC_SPI_SendByte(unsigned char data) {
  UDR2 = data;
}

unsigned char ADC_SPI_ReadByte() {
  return UDR2;
}


unsigned char ADC_SPI_WaitReadByte() {
  ADC_SPI_WaitReceiveBufferFull();
  return ADC_SPI_ReadByte();
}

void ADC_SPI_WaitSendByte(unsigned char sendData) {
  //ADC_SPI_WaitTransmitBufferEmpty(); // transmit buffer should be empty
  ADC_SPI_SendByte(sendData);
}


void EnableADCChipSelect() {
  bit_clear(PORTC,4);
}

void DisableADCChipSelect() {
  bit_set(PORTC,4);
}

#define SPI_PRESCALER 256
#define SPI_CLOCK_RATE_2_COUNTER_START_VALUE(f) (256-F_CPU/SPI_PRESCALER/(f))

typedef union {
	struct {
		byte lowByte;
		byte highByte;
	};
	unsigned short val;
} tADCValue;


/*
  Previously ADC data was read by reading the two data bytes and then
  accumulating the data and incrementing the data read counter.
  ReadADCs uses the time until read data is ready to be read to
  accumulate the channel data and increment the counter.
*/

void ReadADCs() {
#ifdef SPI_MEASURE
  static unsigned char adcloop;
  unsigned long t0 = micros();
#endif

  //bit_set(PORTL,6); // To test performance
  ADC_SPI_WaitTransmitBufferEmpty(); // wait for any pending send
  ADC_SPI_ReadByte();                // discard any pending input byte

  EnableADCChipSelect();

  ADC_SPI_WaitSendByte(adc_cmd[0]);  // Command to read the first channel
  ADC_SPI_WaitReadByte();            // skip the first read byte, as it does not contain ADC data
  ADC_SPI_WaitSendByte(0);           // start read of high byte

  tADCData *p = &ADCData[-1];        // for speed reasons we start at -1 and increment the pointer inside the loop
	tADCValue ADCValue;
  byte cmd = (p+1)->cmd;
  for (uint8_t ch = 0; ch < ADC_SPI_LASTCHANNEL; ch++) {
		ADCValue.highByte = ADC_SPI_WaitReadByte();     // read high byte
		ADC_SPI_WaitSendByte(cmd);                      // select next ADC input

    // after sending a byte, we have 8 SPI clock cycles time,
    // which is 3us at 375ns cycle time
    p++;
    cmd = (p+1)->cmd;
    p->numberOfSamples++;
    if(p->numberOfSamples == 0) {  // during start up the counter could overrun
			p->numberOfSamples = 1;      // this does not happen during normal operation
			p->value = 0;                // so dropping the old values is fine here
		}

		ADCValue.lowByte = ADC_SPI_WaitReadByte(); // read low byte
		if(ch != ADC_SPI_LASTCHANNEL-1) {
	    ADC_SPI_WaitSendByte(0);
		}

    // after sending a byte, we have 8 SPI clock cycles time
    // which is 3us at 375ns cycle time
    p->value += ADCValue.val;
  }

  DisableADCChipSelect();
  //bit_clear(PORTL,6); // To test performance

#ifdef SPI_MEASURE
  unsigned long t1 = micros();

  if(adcloop++ == 0)
  {
    Serial.println(t1-t0);
  }
#endif

  //TCNT2 = SPI_CLOCK_RATE_2_COUNTER_START_VALUE(411);
  TCNT2 = SPI_CLOCK_RATE_2_COUNTER_START_VALUE(1000);
}

ISR (TIMER2_OVF_vect) {
	ReadADCs();
}



void zero_ArduCopter_ADC(void) {
	cli();
  for (byte i=0; i < sizeof(ADCData)/sizeof(ADCData[0]); i++) {
    ADCData[i].value = 0;
    ADCData[i].numberOfSamples = 0;
  }
  sei();
}

void initializeADC(void) {
  zero_ArduCopter_ADC();
  for (byte i=0; i < sizeof(ADCData)/sizeof(ADCData[0]); i++) {
		ADCData[i].cmd = adc_cmd[i+1];
	}
	ADCData[ADC_SPI_LASTCHANNEL-1].cmd = 0;


  pinMode(ADC_CHIP_SELECT,OUTPUT);
  DisableADCChipSelect();

  // Setup Serial Port2 in SPI mode
  UBRR2 = 0;
  DDRH |= (1<<PH2);  // SPI clock XCK2 (PH2) as output. This enables SPI Master mode

  // Set MSPI mode of operation and SPI data mode 0.
  UCSR2C = (1<<UMSEL21) | (1<<UMSEL20); //|(0<<UCPHA2)|(0<<UCPOL2);

  // Enable receiver and transmitter.
  UCSR2B = (1<<RXEN2) | (1<<TXEN2);

  // Set Baud rate
  UBRR2 = 2; // SPI clock running at 2.6MHz
//  UBRR2 = 1; // SPI clock running at 4MHz
//  UBRR2 = 0; // SPI clock running at 8MHz


  // Enable Timer2 Overflow interrupt to capture ADC data
  TIMSK2 = 0;                   // Disable interrupts
  TCCR2A = 0;                   // normal counting mode
  TCCR2B = _BV(CS21) | _BV(CS22); // Set prescaler of 256
  TCNT2  = 0;
  TIFR2  = _BV(TOV2);           // clear pending interrupts;
  TIMSK2 = _BV(TOIE2);          // enable the overflow interrupt
}

int readADC(unsigned char ch_num) {
  int result;

	tADCData *p = &ADCData[ch_num];

  cli();  // We stop interrupts to read the variables
  if (p->numberOfSamples > 0)
	  result = p->value/(p->numberOfSamples*8);
  else
	  result = 0;

  p->value = 0;    // Initialize for next reading
  p->numberOfSamples = 0;
  sei();

  return(result);
}

#endif // #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)

#endif //#define _APM_ADC_OPTIMIZED_H_
