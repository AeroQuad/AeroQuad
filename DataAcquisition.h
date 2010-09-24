/*
  AeroQuad v2.1 - September 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

// This header file defines function calls and ISR's needed to communicatw
// over SPI, I2C and other bus communication protocols for measuring sensor data

// *******************************************
// SPI Communication for APM ADC
// Code written by: Jordi Munoz and Jose Julio
// *******************************************
#ifdef ArduCopter
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
const unsigned char adc_cmd[9]=  { 0x87, 0xC7, 0x97, 0xD7, 0xA7, 0xE7, 0xB7, 0xF7, 0x00 };
volatile long adc_value[8];
volatile unsigned char adc_counter[8];

unsigned char ADC_SPI_transfer(unsigned char data) {
  /* Wait for empty transmit buffer */
  while ( !( UCSR2A & (1<<UDRE2)) );
  /* Put data into buffer, sends the data */
  UDR2 = data;
  /* Wait for data to be received */
  while ( !(UCSR2A & (1<<RXC2)) );
  /* Get and return received data from buffer */
  return UDR2;
}

ISR (TIMER2_OVF_vect) {
  uint8_t ch;
  unsigned int adc_tmp;
  
  //bit_set(PORTL,6); // To test performance
  bit_clear(PORTC,4);             // Enable Chip Select (PIN PC4)
  ADC_SPI_transfer(adc_cmd[0]);       // Command to read the first channel
  for (ch=0;ch<7;ch++) {
    adc_tmp = ADC_SPI_transfer(0)<<8;    // Read first byte
    adc_tmp |= ADC_SPI_transfer(adc_cmd[ch+1]);  // Read second byte and send next command
    adc_value[ch] += adc_tmp>>3;     // Shift to 12 bits
    adc_counter[ch]++;               // Number of samples
    }
  bit_set(PORTC,4);                // Disable Chip Select (PIN PC4)
  //bit_clear(PORTL,6); // To test performance
  TCNT2 = 104;        // 400 Hz
}

void initialize_ArduCopter_ADC(void) {
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

int analogRead_ArduCopter_ADC(unsigned char ch_num) {
  int result;
  cli();  // We stop interrupts to read the variables
  if (adc_counter[ch_num]>0)
	result = adc_value[ch_num]/adc_counter[ch_num];
  else
	result = 0;
  adc_value[ch_num] = 0;    // Initialize for next reading
  adc_counter[ch_num] = 0;
  sei();
  return(result);
}
  
void zero_ArduCopter_ADC(void) {
  for (byte n; n<8; n++) {
    adc_value[n] = 0;
    adc_counter[n] = 0;
  }
}
#endif

// ********************************************
// I2C Communication with Wii Sensors
// Original code written by lamarche_mathieu
// Modifications by jihlein 
// ********************************************
// I2C function calls defined in I2C.h

short NWMP_acc[3];
short NWMP_gyro[3];

void Init_Gyro_acc();
void updateControls();

void Init_Gyro_Acc(void) {
  //Init WM+ and Nunchuk
  updateRegisterI2C(0x53, 0xFE, 0x05);
  delay(100);
  updateRegisterI2C(0x53, 0xF0, 0x55);
  delay(100);
};

void updateControls() {
  int i,j;
  unsigned char buffer[6];

  for(j=0;j<2;j++) {
    sendByteI2C(0x52, 0x00);
    Wire.requestFrom(0x52,6);
    for(i = 0; i < 6; i++) 
      buffer[i] = Wire.receive();
    if (buffer[5] & 0x02) { //If WiiMP
      NWMP_gyro[0]= (((buffer[4]>>2)<<8) +  buffer[1])/16;  //hji
      NWMP_gyro[1]= (((buffer[5]>>2)<<8) +  buffer[2])/16;  //hji
      NWMP_gyro[2]=-(((buffer[3]>>2)<<8) +  buffer[0])/16;  //hji
    }
    else {//If Nunchuk
      NWMP_acc[0]=(buffer[2]<<1)|((buffer[5]>>4)&0x01);  //hji
      NWMP_acc[1]=(buffer[3]<<1)|((buffer[5]>>5)&0x01);  //hji
      NWMP_acc[2]=buffer[4];                             //hji
      NWMP_acc[2]=NWMP_acc[2]<<1;                        //hji
      NWMP_acc[2]=NWMP_acc[2] & 0xFFFC;                  //hji
      NWMP_acc[2]=NWMP_acc[2]|((buffer[5]>>6)&0x03);     //hji
    }
  }
}

