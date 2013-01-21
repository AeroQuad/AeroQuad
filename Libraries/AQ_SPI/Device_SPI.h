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

#ifndef _AEROQUAD_DEVICE_SPI_H_
#define _AEROQUAD_DEVICE_SPI_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

//SPI pins on Mega2560:
#define DOUT     51 //MOSI
#define DIN      50 //MISO
#define SCK      52 //SCLK
#define OSD_CS   22 //SS_OSD on AeroQuad v2.x shield

void spi_osd_select() {
  digitalWrite( OSD_CS, LOW );
}

void spi_osd_deselect() {
  digitalWrite( OSD_CS, HIGH );
}

void initializeSPI() {

  pinMode( 53, OUTPUT ); //Default CS pin - needs to be output or SPI peripheral will behave as a slave instead of master
  pinMode( DOUT, OUTPUT );
  pinMode( DIN, INPUT );
  pinMode( SCK, OUTPUT );

  pinMode( OSD_CS, OUTPUT );
  digitalWrite( OSD_CS, HIGH );

  // SPCR = 01010000
  // interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  // sample on leading edge of clk,system clock/4 rate (fastest)
  SPCR = (1 << SPE) | (1 << MSTR);
  SPSR; // dummy read from HW register
  SPDR; // dummy read from HW register
  delay( 10 );

}


//////////////////////////////////////////////////////////////
//Performs an 8-bit SPI transfer operation
byte spi_transfer( byte data ) {

  SPDR = data; //transfer data with hardware SPI
  while ( !(SPSR & _BV(SPIF)) ) ;
  return SPDR;
}

void spi_writereg(byte r, byte d) {

  spi_transfer(r);
  spi_transfer(d);
}

byte spi_readreg(byte r) {

  spi_transfer(r);
  return spi_transfer(0);
}

#endif // Mega1280/2560

#if defined(AeroQuadSTM32)

HardwareSPI device_spi(2); // SPI2 on STM32; wired on header

#define OSD_CS    Port2Pin('A', 3) // pin 26 == 'SVR0' pin on AQ32 (TIM5_CH4), may need to be changed...

void spi_osd_select() {
  digitalWrite( OSD_CS, LOW );
}

void spi_osd_deselect() {
  digitalWrite( OSD_CS, HIGH );
}


void initializeSPI() {

  pinMode( OSD_CS, OUTPUT );
  digitalWrite( OSD_CS, HIGH );

  device_spi.begin(SPI_9MHZ, MSBFIRST, 0);
}


void spi_writereg(byte r, byte d) {

  device_spi.transfer(r);
  device_spi.transfer(d);  // use transfer to ensure we don't return until bus is free
}

byte spi_readreg(byte r) {

  device_spi.transfer(r);
  return(device_spi.transfer(0));
}


#endif // AeroQuadSTM32
#endif
