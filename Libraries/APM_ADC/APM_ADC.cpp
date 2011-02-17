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

#include "APM_ADC.h"

#include <inttypes.h>
#include <avr/interrupt.h>
#include "WConstants.h"

#define bit_set(p,m) ((p) |= (1<<m)) 
#define bit_clear(p,m) ((p) &= ~(1<<m))

// We use Serial Port 2 in SPI Mode
#define ADC_DATAOUT 51        // MOSI
#define ADC_DATAIN  50        // MISO 
#define ADC_SPICLOCK  52      // SCK
#define ADC_CHIP_SELECT 33    // PC4   9 // PH6  Puerto:0x08 Bit mask : 0x40

// Commands for reading ADC channels on ADS7844
const unsigned char _adcCmd[9]=  { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };
volatile long _adcValue[8];
volatile unsigned char _adcCounter[8];

ISR (TIMER2_OVF_vect) 
{
  uint8_t ch;
  unsigned int adc_tmp;
  
  //bit_set(PORTL,6); // To test performance
  bit_clear(PORTC,4);             // Enable Chip Select (PIN PC4)
  ADCSPItransfer(_adcCmd[0]);       // Command to read the first channel
  for (ch=0;ch<7;ch++) 
  {
    adc_tmp = ADCSPItransfer(0)<<8;    // Read first byte
    adc_tmp |= ADCSPItransfer(_adcCmd[ch+1]);  // Read second byte and send next command
    _adcValue[ch] += adc_tmp>>3;     // Shift to 12 bits
    _adcCounter[ch]++;               // Number of samples
  }
  bit_set(PORTC,4);                // Disable Chip Select (PIN PC4)
  //bit_clear(PORTL,6); // To test performance
  TCNT2 = 104;        // 400 Hz
}

unsigned char ADCSPItransfer(unsigned char data) 
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR2A & (1<<UDRE2)) );
  /* Put data into buffer, sends the data */
  UDR2 = data;
  /* Wait for data to be received */
  while ( !(UCSR2A & (1<<RXC2)) );
  /* Get and return received data from buffer */
  return UDR2;
}

void initializeADC() 
{
  unsigned char tmp;
  
  pinMode(ADC_CHIP_SELECT,OUTPUT);
  
  digitalWrite(ADC_CHIP_SELECT,HIGH); // Disable device (Chip select is active low)

  // Setup Serial Port2 in SPI mode
  UBRR2 = 0;   
  DDRH |= (1<<PH2);  // SPI clock XCK2 (PH2) as output. This enable SPI Master mode
  // Set MSPI mode of operation and SPI data mode 0.
  UCSR2C = (1<<UMSEL21)|(1<<UMSEL20); //|(0<<UCPHA2)|(0<<UCPOL2);
  // Enable receiver and transmitter.
  UCSR2B = (1<<RXEN2)|(1<<TXEN2);
  // Set Baud rate
  UBRR2 = 2; // SPI clock running at 2.6MHz

  // Enable Timer2 Overflow interrupt to capture ADC data
  TIMSK2 = 0;  // Disable interrupts 
  TCCR2A = 0;  // normal counting mode 
  TCCR2B = _BV(CS21)|_BV(CS22);     // Set prescaler of 256
  TCNT2 = 0;
  TIFR2 = _BV(TOV2);  // clear pending interrupts; 
  TIMSK2 =  _BV(TOIE2) ; // enable the overflow interrupt
}

int readADC(unsigned char ch_num) 
{
  int result;
  cli();  // We stop interrupts to read the variables
  if (_adcCounter[ch_num]>0)
  {
    result = _adcValue[ch_num]/_adcCounter[ch_num];
  }
  else
  {
    result = 0;
  }
  _adcValue[ch_num] = 0;    // Initialize for next reading
  _adcCounter[ch_num] = 0;
  sei();
  return(result);
}
  
void zeroADC() 
{
  for (byte n; n<8; n++) 
  {
    _adcValue[n] = 0;
    _adcCounter[n] = 0;
  }
}

#endif
