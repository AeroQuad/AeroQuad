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

/*
 This module provides on screen display (OSD) for Aeroquad FPV flying.

 It can display
  - artificial horizon with pitch lines
  - battery information
  - altitude (and altitude hold state and target )
  - compass heading
  - flight timer
  - callsign
  - RSSI information
  - additional notification strings

 You will need to upload a special character set initially using the
 provided MAX7456_Font_Updater sketch.

 The user must connect a MAX7456 OSD chip to the appropriate header pins on
 the Arduino. These pins are marked 'OSD' on the AeroQuad Shield v2.

 If the chip is not connected properly, this code may hang.

 If using the SparkFun MAX7456 breakout board, the reset pin should be wired
 high (+5V) either directly or with 10kOhm resistor.

 As the MAX7456 may draw up to 100mA it is a good idea to power it using
 separate regulator (or power it from one of the BEC:s on ESCs). It is known
 that powering from arduino and using 3S battery will overheat the regulator
 which will lead to crash.

 Special thanks to Alamo for contributing this capability!

 */

#ifndef _AQ_OSD_MAX7456_H_
#define _AQ_OSD_MAX7456_H_

#include <stdio.h>
#include <stdarg.h>

#include "OSD.h"
#include "GlobalDefined.h"

// You can configure positioning of various display elements below.
// '#defines' for elements which will not be displayed, can be ignored.
//
// The MAX7456 overlays characters in a grid 30 characters wide, 16/13 high
// (PAL/NTSC respectively). The row/column defines below correspond to
// positions in the grid of characters, with the origin at the top left.
// 0-origin indexing is used - ie row 0, col 0 is the highest, leftmost
// character on the screen while row 15, col 29 is the bottom right (for PAL).
//
// Generally avoid using the extreme border rows/columns as they are not
// always visible.
//
// Display elements start at the position you give and print to the right.
// They will wrap around to the next row if there are too few columns remaining
// on the row you specify.

//Battery info - 5-16 characters long
#define VOLTAGE_ROW 2
#define VOLTAGE_COL 1

//Compass reading - 5 characters long
#define COMPASS_ROW 1
#define COMPASS_COL 13

//Altitude reading - up to 8 characters long (32768 max)
#define ALTITUDE_ROW 1
#define ALTITUDE_COL 1

//Flight timer - 6 characters long
#define TIMER_ROW 1
#define TIMER_COL 23

//Callsign
#define CALLSIGN_ROW 2
#define CALLSIGN_COL 23
#ifdef ShowCallSign
const char *callsign = "AeroQD";
#endif

// RSSI monitor
#define RSSI_ROW     3
#define RSSI_COL     23
#define RSSI_PIN     A6     // analog pin to read
#define RSSI_RAWVAL         // show raw A/D value instead of percents (for tuning)
#define RSSI_100P    1023   // A/D value for 100%
#define RSSI_0P      0      // A/D value for 0%
#define RSSI_WARN    20     // show alarm at %

// Notify
#define NOTIFY_ROW MAX_screen_rows-3
#define NOTIFY_COL 1 // don't change this, it needs a full line

/********************** End of user configuration section ********************************/


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

//configuration for AI
#define LINE_ROW_0 0x80                             // character address of a character with a horizontal line in row 0. Other rows follow this one
#define AI_MAX_PITCH_ANGLE (PI/4)                   // bounds of scale used for displaying pitch. When pitch is >= |this number|, the pitch lines will be at top or bottom of bounding box
static const byte ROLL_COLUMNS[4] = {10,12,17,19};  // columns where the roll line is printed
#define PITCH_L_COL 7
#define PITCH_R_COL 22
#define AI_DISPLAY_RECT_HEIGHT 9                    // Height of rectangle bounding AI. Should be odd so that there is an equal space above/below the centre reticle

#define RETICLE_ROW (MAX_screen_rows/2)             // centre row - don't change this
#define RETICLE_COL 14                              // reticle will be in this col, and col to the right

#define AI_TOP_PIXEL ((RETICLE_ROW - AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_BOTTOM_PIXEL ((RETICLE_ROW + AI_DISPLAY_RECT_HEIGHT/2)*18)
#define AI_CENTRE (RETICLE_ROW*18+10)               // row, in pixels, corresponding to zero pitch/roll.

// void notifyOSD(byte flags, char *fmt, ...)
//   - display notification string on OSD
//
// void notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, char *fmt, ...)
//   - display notification with blinking region = 'cursor'
//   - characters between cursorLeft and cursorRight will blink if OSD_CURSOR flag is used
//
//   fmt == NULL will clear
//   flags -- message priority and options i.e. (OSD_CRIT|OSD_BLINK|OSD_CENTER)
#define OSD_INFO    0x00
#define OSD_WARN    0x40
#define OSD_ERR     0x80
#define OSD_CRIT    0xc0
#define OSD_NOCLEAR 0x20 // do not clear the message after ~5s
#define OSD_CURSOR  0x10 // enable cursor
#define OSD_BLINK   0x08 // blinking message
#define OSD_INVERT  0x04 // inverted message
#define OSD_NOW     0x02 // show message immediately
#define OSD_CENTER  0x01 // Justify at center


byte osdNotificationTimeToShow = 0;
byte osdNotificationFlags      = 0;
byte osdNotificationCursorL    = 0;
byte osdNotificationCursorR    = 0;
char osdNotificationBuffer[29];     // do not change this

#define notifyOSD(flags,fmt,args...) notifyOSDmenu(flags,255,255,fmt, ## args)

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

byte notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, const char *fmt, ...) {

  va_list ap;
  if ((osdNotificationTimeToShow > 0) && ((flags >> 6) < (osdNotificationFlags >> 6))) {
    return 1; // drop message, we tell it to caller
  }
  if (fmt == NULL) {
    // clear
    memset(osdNotificationBuffer, 0, 28);
    osdNotificationFlags = 0;
  }
  else {
    osdNotificationFlags = flags; // will set priority and flags
    osdNotificationTimeToShow = (flags & OSD_NOCLEAR) ? 255 : 50; // set timeout for message
    va_start(ap, fmt);
    byte len=vsnprintf(osdNotificationBuffer, 29, fmt, ap);
    va_end(ap);
    if (len > 28) {
      len = 28;
    }
    memset(osdNotificationBuffer+len, 0, 28-len);
    if (flags & OSD_CENTER) {
      byte i = (28 - len) / 2;
      if (i) {
        // move text right to center it
        memmove(osdNotificationBuffer + i, osdNotificationBuffer, strlen(osdNotificationBuffer));
        memset(osdNotificationBuffer, 0, i);
        // adjust cursor position also if needed
        if (flags & OSD_CURSOR) {
          cursorLeft  += i;
          cursorRight += i;
        }
      }
    }
    osdNotificationCursorL = cursorLeft < 27 ? cursorLeft : 27;
    osdNotificationCursorR = cursorRight < 27 ? cursorRight : 27;
  }
  if (flags & OSD_NOW) {
    displayNotify();
  }
  else {
    osdNotificationFlags |= OSD_NOW; // this will tell next update to show message
  }
  return 0;
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

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Battery voltage Display //////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef BattMonitor

#include "BatteryMonitorTypes.h"

byte    osdBatCounter = 0;
boolean descentWarningShown = false;

void displayVoltage(byte areMotorsArmed) {

  byte    osdBatNo     = osdBatCounter % numberOfBatteries;
  boolean osdBatMinMax = osdBatCounter / numberOfBatteries / 4;

  // only show min/max values when not armed
  if (areMotorsArmed == true) {
    osdBatMinMax = false;
  }

  int currentValue;
  if (osdBatMinMax) {
    currentValue = batteryData[osdBatNo].minVoltage*10.0;
  }
  else {
    currentValue = batteryData[osdBatNo].voltage*10.0;
  }

  char buf[12];
  snprintf(buf,7,"%c%2d.%1dV",(osdBatMinMax) ? '\23' : '\20', currentValue/10,currentValue%10);

  // Following blink only symbol on warning and all on alarm
  writeChars( buf,   1, batteryIsWarning(osdBatNo)?1:0, VOLTAGE_ROW + osdBatNo, VOLTAGE_COL );
  writeChars( buf+1, 5, batteryIsAlarm(osdBatNo)?1:0,   VOLTAGE_ROW + osdBatNo, VOLTAGE_COL + 1 );

  if (batteryData[osdBatNo].cPin != BM_NOPIN) {
    // current sensor installed
    if (osdBatMinMax) {
      currentValue = batteryData[osdBatNo].maxCurrent*10.0;
    }
    else {
      currentValue = batteryData[osdBatNo].current*10.0;
    }
    
    if (abs(currentValue)>=100) { // > 10A only display whole amps
      snprintf(buf,12,"%4dA%5d\24  ", currentValue/10, (int)batteryData[osdBatNo].usedCapacity);
    }
    else {
      snprintf(buf,12,"%c%1d.%1dA%5d\24  ", currentValue<0?'-':' ',abs(currentValue/10),abs(currentValue%10),(int)batteryData[osdBatNo].usedCapacity);
    }      
      
    writeChars( buf, 11, 0, VOLTAGE_ROW+osdBatNo, VOLTAGE_COL+6 );
  }

  osdBatCounter++;
  if (osdBatCounter >= numberOfBatteries * 8) {
    osdBatCounter = 0;
  }
  
  #if defined (BattMonitorAutoDescent)
    if (batteryAlarm && areMotorsArmed) {
      if (!descentWarningShown) {
        notifyOSD(OSD_CENTER|OSD_CRIT|OSD_BLINK, "BAT. CRITICAL - DESCENTING");
        descentWarningShown = true;
      }
    }
    else {
      descentWarningShown = false;
    }
  #endif
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// AltitudeHold Display /////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

int lastAltitude     = 12345;     // bogus initial values to force update
int lastHoldAltitude = 12345;
byte lastHoldState   = 6;

void displayAltitude(float readedAltitude, float desiredAltitudeToKeep, boolean altHoldState) {
  #ifdef feet
    int currentAltitude = readedAltitude*3.281;
    int currentHoldAltitude = desiredAltitudeToKeep*3.281;
  #else // metric
    int currentAltitude = readedAltitude*10.0; // 0.1m accuracy!!
    int currentHoldAltitude = desiredAltitudeToKeep*10.0;
  #endif
  char buf[7];

  if ( lastAltitude != currentAltitude ) {
    #ifdef feet
      snprintf(buf,7,"\10%4df",currentAltitude);
    #else
      if (abs(currentAltitude)<100) {
        snprintf(buf,7,"\010%c%1d.%1dm",currentAltitude < 0 ? '-' : ' ', abs(currentAltitude/10),abs(currentAltitude%10));
      }
      else {
        snprintf(buf,7,"\010%4dm",currentAltitude/10);
      }
    #endif
    writeChars( buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL );
    lastAltitude = currentAltitude;
  }

  // AltitudeHold handling:
  // - show hold altitude when it is active
  // - show "panic" if 'paniced' out
  boolean isWriteNeeded = false;
  switch (altHoldState) {
  case OFF:
    if (lastHoldState != OFF) {
      lastHoldState = OFF;
      memset(buf,0,6);
      isWriteNeeded = true;
    }
    break;
  case ON:
    if ((lastHoldState != ON) || (lastHoldAltitude != currentHoldAltitude)) {
      lastHoldState = ON;
      lastHoldAltitude=currentHoldAltitude;
      #ifdef feet
        snprintf(buf,7,"\11%4df",currentHoldAltitude);
      #else
        if (abs(currentHoldAltitude)<100) {
          snprintf(buf,7,"\011%c%1d.%1dm", currentHoldAltitude < 0 ? '-' : ' ',abs(currentHoldAltitude/10),abs(currentHoldAltitude%10));
        }
        else {
          snprintf(buf,7,"\011%4dm",currentHoldAltitude/10);
        }
      #endif
      isWriteNeeded = true;
    }
    break;
  case ALTPANIC:
    if (lastHoldState != ALTPANIC) {
      lastHoldState = ALTPANIC;
      snprintf(buf,7,"\11panic");
      isWriteNeeded = true;
    }
    break;
  }

  if (isWriteNeeded) {
    writeChars( buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL+6 );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// HeadingMagHold Display ///////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef HeadingMagHold

int lastHeading = 361; // bogus to force update

void displayHeading(int currentHeading) {
  
  if (currentHeading != lastHeading) {
    char buf[6];
    snprintf(buf,6,"\6%3d\7",currentHeading); // \6 is compass \7 is degree symbol
    writeChars( buf, 5, 0, COMPASS_ROW, COMPASS_COL );
    lastHeading = currentHeading;
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Flight time Display //////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef ShowFlightTimer

unsigned long prevTime = 0;           // previous time since start when OSD.update() ran
unsigned int prevArmedTimeSecs = 111; // bogus to force update
unsigned long armedTime = 0;          // time motors have spent armed

void displayFlightTime(byte areMotorsArmed) {
  if (areMotorsArmed == ON) {
    armedTime += ( currentTime-prevTime );
  }

  prevTime = currentTime;
  unsigned int armedTimeSecs = armedTime / 1000000;
  if (armedTimeSecs != prevArmedTimeSecs) {
    prevArmedTimeSecs = armedTimeSecs;
    char buf[7];
    snprintf(buf,7,"\5%02u:%02u",armedTimeSecs/60,armedTimeSecs%60);
    writeChars(buf, 6, 0, TIMER_ROW, TIMER_COL );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)
#ifdef ShowRSSI

short lastRSSI = 1234; //forces update at first run

void displayRSSI() {

  int val = analogRead(RSSI_PIN);
  #ifndef RSSI_RAWVAL
    val = (val - RSSI_0P) * 100 / (RSSI_100P - RSSI_0P);
    if (val < 0) {
      val = 0;
    }
    if (val > 100) {
      val = 100;
    }
  #endif
  if (val != lastRSSI) {
    lastRSSI = val;
    char buf[6];
    #ifdef RSSI_RAWVAL
      snprintf(buf, 6, "\372%4u", val);
      writeChars(buf, 5, 0, RSSI_ROW, RSSI_COL);
    #else
      snprintf(buf, 6, "\372%3u%%", val);
      writeChars(buf, 5, (RSSI_WARN>val)?1:0, RSSI_ROW, RSSI_COL);
    #endif
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// ATTITUDE Display /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef ShowAttitudeIndicator

byte AIoldline[5] = {0,0,0,0,0};

void displayArtificialHorizon(float roll, float pitch) {

  short AIrows[5]   = {0,0,0,0,0};  //Holds the row, in pixels, of AI elements: pitch then roll from left to right.
  //Calculate row of new pitch lines
  AIrows[0] = constrain( (int)AI_CENTRE + (int)((pitch/AI_MAX_PITCH_ANGLE)*(AI_CENTRE-AI_TOP_PIXEL) ), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );  //centre + proportion of full scale
  char pitchLine = LINE_ROW_0 + (AIrows[0] % 18);

  if (AIoldline[0] != AIrows[0]/18) {
    //Remove old pitch lines if not overwritten by new ones
    writeChars( NULL, 1, 0, AIoldline[0], PITCH_L_COL );
    writeChars( NULL, 1, 0, AIoldline[0], PITCH_R_COL );
    AIoldline[0] = AIrows[0]/18;
  }

  //Write new pitch lines
  writeChars( &pitchLine, 1, 0, AIoldline[0], PITCH_L_COL );
  writeChars( &pitchLine, 1, 0, AIoldline[0], PITCH_R_COL );

  //Calculate row (in pixels) of new roll lines
  int distFar  = (ROLL_COLUMNS[3] - (RETICLE_COL + 1))*12 + 6; //horizontal pixels between centre of reticle and centre of far angle line
  int distNear = (ROLL_COLUMNS[2] - (RETICLE_COL + 1))*12 + 6;
  float gradient = 1.4 * roll; // was "tan(roll)", yes rude but damn fast !!
  AIrows[4] = constrain( AI_CENTRE - (int)(((float)distFar)*gradient), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[3] = constrain( AI_CENTRE - (int)(((float)distNear)*gradient), AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[1] = constrain( 2*AI_CENTRE - AIrows[4], AI_TOP_PIXEL, AI_BOTTOM_PIXEL );
  AIrows[2] = constrain( 2*AI_CENTRE - AIrows[3], AI_TOP_PIXEL, AI_BOTTOM_PIXEL );

  //writing new roll lines to screen
  for (byte i=1; i<5; i++ ) {
    // clear previous roll lines if not going to overwrite
    if (AIoldline[i] != AIrows[i]/18) {
      writeChars( NULL, 1, 0, AIoldline[i], ROLL_COLUMNS[i-1] );
      AIoldline[i] =  AIrows[i]/18;
    }
    //converting rows (in pixels) to character addresses used for the 'lines'
    char RollLine = LINE_ROW_0 + (AIrows[i] % 18);
    writeChars( &RollLine, 1, 0, AIoldline[i], ROLL_COLUMNS[i-1] );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Reticle Display //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Reticle on the center of the screen
// We have two reticles empty one for RATE_FLIGHT_MODE and one with (s) for 'ATTITUDE_FLIGHT_MODE' mode
#ifdef ShowReticle

byte lastFlightMode = 9;

void displayReticle(byte flightMode) {

  if (lastFlightMode != flightMode) {
    writeChars( (flightMode == 0) ? "\1\2" : "\3\4", 2, 0, RETICLE_ROW, RETICLE_COL ); //write 2 chars to row (middle), column 14
    lastFlightMode = flightMode;
  }
}
#endif


byte displayNotify(){

  if (osdNotificationFlags & OSD_NOW) {
    osdNotificationFlags &= ~OSD_NOW;
    // we have new message to show

    if ((osdNotificationFlags&OSD_CURSOR) && (osdNotificationCursorL!=255)) {

      if (osdNotificationCursorL > 0) {
        writeChars(osdNotificationBuffer,osdNotificationCursorL,
          ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?1:0),
          NOTIFY_ROW,NOTIFY_COL);
      }

      writeChars(osdNotificationBuffer+osdNotificationCursorL,osdNotificationCursorR-osdNotificationCursorL+1,
        ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?0:1),
        NOTIFY_ROW,NOTIFY_COL+osdNotificationCursorL);

      if (osdNotificationCursorR < 27) {
        writeChars(osdNotificationBuffer+osdNotificationCursorR+1,27-osdNotificationCursorR,
          ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?1:0),
          NOTIFY_ROW,NOTIFY_COL+osdNotificationCursorR+1);
      }
    }
    else {
      writeChars(osdNotificationBuffer,28,
        ((osdNotificationFlags&OSD_INVERT)?2:0)|((osdNotificationFlags&OSD_BLINK)?1:0),
        NOTIFY_ROW,NOTIFY_COL);
    }
    return 1;
  }

  if ((osdNotificationTimeToShow > 0) && (osdNotificationTimeToShow != 255)) {
    if (osdNotificationTimeToShow-- == 1) {
      writeChars(NULL,28,0,NOTIFY_ROW,NOTIFY_COL);
      return 1;
    }
  }
  return 0; // nothing was done
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

  #ifdef ShowCallSign
    writeChars(callsign,strlen(callsign),0,CALLSIGN_ROW,CALLSIGN_COL);
  #endif

  // show notification of active video format
  notifyOSD(OSD_NOW, "VIDEO: %s", (DISABLE_display) ? "PAL" : "NTSC");
}

#endif  // #define _AQ_OSD_MAX7456_H_


