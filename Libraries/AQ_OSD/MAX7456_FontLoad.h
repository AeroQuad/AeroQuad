/*
  AeroQuad v3.2 - Jan 2013
 www.AeroQuad.com
 Copyright (c) 2013 AeroQuad team.  All rights reserved.
 An Open Source Arduino/STM32 based multicopter.

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

#ifndef _AQ_OSD_MAX7456_FONT_LOAD_H_
#define _AQ_OSD_MAX7456_FONT_LOAD_H_

#if !defined(AeroQuadSTM32)
  #error You must use separate MAX7456_FontLoader sketch on Arduino
#endif

#define MAX_font_rom 0xff
#define STATUS_reg_nvr_busy 0x20
#define NVM_ram_size 0x36
#define WRITE_nvr 0xa0
#define READ_nvr 0x50

// Include actual font data
#define PROGMEM // discard progmem define
#include <MAX7456_Font.h>

void write_NVM_character(byte ch, const byte* addr)
{
  byte x;
  // disable display
  spi_osd_select();
  spi_writereg(VM0,DISABLE_display);
  spi_writereg(CMAH,ch);  // select character
  for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) bytes of character to shadow ram
  {
    spi_writereg(CMAL,x); // set offset inside char
    spi_writereg(CMDI,*(addr+x));
  }
  // transfer a 54 bytes from shadow ram to NVM, takes about 12ms to complete
  spi_writereg(CMM,WRITE_nvr);

  do {
    delay(1);
    Serial.print(".");
  } while (spi_readreg(STAT) & STATUS_reg_nvr_busy);

  spi_writereg(VM0,ENABLE_display_vert);
  spi_osd_deselect();
}

void read_NVM_character(byte ch, byte* addr)
{
  byte x;
  // disable display
  spi_osd_select();
  spi_writereg(VM0,DISABLE_display);
  spi_writereg(CMAH,ch);  // select character
  // transfer a 54 bytes from NVM to shadow RAM
  spi_writereg(CMM,READ_nvr);

  for(x = 0; x < NVM_ram_size; x++) // read out 54 (out of 64) bytes of character from shadow ram
  {
    spi_writereg(CMAL,x); // set offset insinde char
    *(addr+x) = spi_readreg(CMDO);
  }
  spi_writereg(VM0,ENABLE_display_vert);
  spi_osd_deselect();
}

int fontCompareChar(const byte *a, const byte *b) {

  for (int i=0; i<54; i++) {
    if ( *(a + i) != *(b + i) ) {
      return 1;
    }
  }
  return 0;
}

void max7456LoadFont()
{
  byte chbuf[54], status;
  if (sizeof(fontdata) != 16384) {
    Serial.println("ERROR: fontdata with invalid size, aborting!!!");
    return;
  }

  for (int ch=0; ch<256; ch++) {
    spi_osd_select();
    status = spi_readreg(STAT);
    spi_osd_deselect();
    if (!status) {
      Serial.println("ERROR: MAX7456 not connected ???");
      return;
    }
    Serial.print("Verifying character:");
    Serial.print((int)ch);
    read_NVM_character(ch,chbuf);
    if (fontCompareChar(chbuf,fontdata+64*ch)) {
      Serial.print("UPLOAD");
      write_NVM_character(ch,fontdata+64*ch);
    } else {
      Serial.print("OK");
    }
    Serial.println();
  }
  Serial.println("Font updated");
}

#endif  // #define _AQ_OSD_MAX7456_FONTLOAD_H_
