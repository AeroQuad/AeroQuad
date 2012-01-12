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

#ifndef _AQ_OSD_MAX7456_RSSI_H_
#define _AQ_OSD_MAX7456_RSSI_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)

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

#endif  // #define _AQ_OSD_MAX7456_RSSI_H_
