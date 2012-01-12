/*
  AeroQuad v3.0 - Nov 2011
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

#ifndef _AQ_OSD_MAX7456_BASE_H_
#define _AQ_OSD_MAX7456_BASE_H_

//MAX7456 register write addresses - see datasheet for lots of info
#define DMM   0x04 //Display memory mode register - for choosing 16bit/8bit write mode, clearing display memory, enabling auto-increment
#define DMAH  0x05 //Holds MSB of display memory address, for setting location of a character on display
#define DMAL  0x06 //Holds remaining 8 bits of display memory address - 480 characters displayed -> 9 bits req'd for addressing
#define DMDI  0x07 //Display memory data in - character address or attribute byte, depending on 8b/16b mode and DMAH[1]
#define VM0   0x00 //Video mode 0 register - for choosing, NTSC/PAL, sync mode, OSD on/off, reset, VOUT on/off
#define VM1   0x01 //Video mode 1 register - nothing very interesting in this one
#define RB0   0x10 //Row 0 brightness register - 15 more follow sequentially (ending at 0x1F)
#define STAT  0xA2 //Status register read address

//MAX7456 commands - provided in datasheet.
#define CLEAR_display      0x04
#define CLEAR_display_vert 0x06
#define END_string         0xff

#define WHITE_level_90     0x02

unsigned MAX_screen_size = 0;
unsigned MAX_screen_rows = 0;
byte ENABLE_display      = 0;
byte ENABLE_display_vert = 0;
byte MAX7456_reset       = 0;
byte DISABLE_display     = 0;

// void writeChars( const char* buf, byte len, byte flags, byte y, byte x )
//
// Writes 'len' character address bytes to the display memory corresponding to row y, column x
// - uses autoincrement mode when writing more than one character
// - will wrap around to next row if 'len' is greater than the remaining cols in row y
// - buf=NULL can be used to write zeroes (clear)
// - flags: 0x01 blink, 0x02 invert (can be combined)
void writeChars( const char* buf, byte len, byte flags, byte y, byte x ) {

  unsigned offset = y * 30 + x;
  spi_select();
  // 16bit transfer, transparent BG, autoincrement mode (if len!=1)
  spi_writereg(DMM, ((flags&1) ? 0x10 : 0x00) | ((flags&2) ? 0x08 : 0x00) | ((len!=1)?0x01:0x00) );

  // send starting display memory address (position of text)
  spi_writereg(DMAH, offset >> 8 );
  spi_writereg(DMAL, offset & 0xff );

  // write out data
  for ( int i = 0; i < len; i++ ) {
    spi_writereg(DMDI, buf==NULL?0:buf[i] );
  }

  // Send escape 11111111 to exit autoincrement mode
  if (len!=1) {
    spi_writereg(DMDI, END_string );
  }
  // finished writing
  spi_deselect();
}

void detectVideoStandard() {

  // First set the default
  boolean pal = false;
  #ifdef PAL
    pal = true;
  #endif
  #ifdef AUTODETECT_VIDEO_STANDARD
    // if autodetect enabled modify the default if signal is present on either standard
    // otherwise default is preserved
    spi_select();
    byte stat=spi_readreg(STAT);
    if (stat & 0x01) {
      pal = true;
    }
    if (stat & 0x02) {
      pal = false;
    }
    spi_deselect();
  #endif

  if (pal) {
    MAX_screen_size=480;
    MAX_screen_rows=16;
    ENABLE_display=0x48;
    ENABLE_display_vert=0x4c;
    MAX7456_reset=0x42;
    DISABLE_display=0x40;
  }
  else {
    MAX_screen_size=390;
    MAX_screen_rows=13;
    ENABLE_display=0x08;
    ENABLE_display_vert=0x0c;
    MAX7456_reset=0x02;
    DISABLE_display=0x00;
  }
}

void initializeOSD() {

  // SPCR = 01010000
  // interrupt disabled,spi enabled,msb 1st,master,clk low when idle,
  // sample on leading edge of clk,system clock/4 rate (fastest)
  SPCR = (1 << SPE) | (1 << MSTR);
  SPSR; // dummy read from HW register
  SPDR; // dummy read from HW register
  delay( 10 );

  detectVideoStandard();

  //Soft reset the MAX7456 - clear display memory
  spi_select();
  spi_writereg( VM0, MAX7456_reset );
  spi_deselect();
  delay( 1 ); //Only takes ~100us typically

  //Set white level to 90% for all rows
  spi_select();
  for( int i = 0; i < MAX_screen_rows; i++ ) {
    spi_writereg( RB0 + i, WHITE_level_90 );
  }

  //ensure device is enabled
  spi_writereg( VM0, ENABLE_display );
  delay(100);
  //finished writing
  spi_deselect();

  OSDsched = 0xff; // This will make everything to be updated next round
  updateOSD();     // Make first update now

  #if defined CALLSIGN
    writeChars(callsign,strlen(callsign),0,CALLSIGN_ROW,CALLSIGN_COL);
  #endif

  // show notification of active video format
  notifyOSD(OSD_NOW, "VIDEO: %s", (DISABLE_display) ? "PAL" : "NTSC");
}

#endif  // #define _AQ_OSD_MAX7456_BASE_H_


