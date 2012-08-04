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

#ifndef _AQ_OSD_MAX7456_CONFIG_H_
#define _AQ_OSD_MAX7456_CONFIG_H_

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
#define VOLTAGE_COL 0

//Compass reading - 5 characters long
#define COMPASS_ROW 1
#define COMPASS_COL 13

//Altitude reading - up to 8 characters long (32768 max)
#define ALTITUDE_ROW 1
#define ALTITUDE_COL 0

//Flight timer - 6 characters long
#define TIMER_ROW 1
#define TIMER_COL 23

//Callsign
#if defined CALLSIGN
  const char *callsign = CALLSIGN;
  #define CALLSIGN_ROW 2
  #define CALLSIGN_COL (29-strlen(callsign))
#endif

// RSSI monitor
#define RSSI_ROW     3
#define RSSI_COL     24
#define SIGNAL_QUALITY_ROW 3
#define SIGNAL_QUALITY_COL 0

// GPS info
#define GPS_ROW     MAX_screen_rows-2
#define GPS_COL     1

// GPS home arrow -- under the reticle
#define GPS_HA_ROW  (MAX_screen_rows/2+2)
#define GPS_HA_COL  14

// Notify
#define NOTIFY_ROW MAX_screen_rows-3
#define NOTIFY_COL 1 // don't change this, it needs a full line

// Artificial horizon mode (default is attitude indicator)
#define ARTIFICIAL_HORIZON

/********************** End of user configuration section ********************************/

#endif  // #define _AQ_OSD_MAX7456_CONFIG_H_
