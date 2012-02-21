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

//OSD pins on AQ v2 shield:
#define CS   22 //SS_OSD
#define DOUT 51 //MOSI
#define DIN  50 //MISO
#define SCK  52 //SCLK


void initializeSPI() {

  pinMode( CS, OUTPUT );
  pinMode( 53, OUTPUT ); //Default CS pin - needs to be output or SPI peripheral will behave as a slave instead of master
  digitalWrite( CS, HIGH );

  pinMode( DOUT, OUTPUT );
  pinMode( DIN, INPUT );
  pinMode( SCK, OUTPUT );
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

void spi_select() {
  digitalWrite( CS, LOW );
}

void spi_deselect() {
  digitalWrite( CS, HIGH );
}



#endif