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

#ifndef _AQ_OSD_H_
#define _AQ_OSD_H_

byte OSDsched = 0;

#ifdef BattMonitor
  void displayVoltage(byte areMotorsArmed);
#endif
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  void displayAltitude(float readedAltitude, float desiredAltitudeToKeep, boolean altitudeHoldState);
#endif
#ifdef HeadingMagHold
  void displayHeading(int currentHeading);
#endif
#ifdef ShowRSSI
  void displayRSSI();
#endif
#ifdef ShowAttitudeIndicator
  void displayArtificialHorizon(float roll, float pitch, byte flightMode);
#endif

void initializeOSD();
void updateOSD();
void displayFlightTime(byte areMotorsArmed);
byte displayNotify();

// OSD notification system
//
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
#define OSD_NOW     0x02 // show message immediately (do not wait until OSD update)
#define OSD_CENTER  0x01 // Justify at center

#define notifyOSD(flags,fmt,args...) notifyOSDmenu(flags,255,255,fmt, ## args)
byte notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, const char *fmt, ...);

#endif
