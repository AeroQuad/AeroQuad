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

#ifndef _AQ_ANALOG_RSSI_READER_H_
#define _AQ_ANALOG_RSSI_READER_H_


#define RSSI_PIN     A6     // analog pin to read
//#define RSSI_RAWVAL         // show raw A/D value instead of percents (for tuning)
#define RSSI_100P    800   // A/D value for 100%
#define RSSI_0P      80      // A/D value for 0%
#define RSSI_WARN    20     // show alarm at %


//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)

short rssiRawValue = 0; //forces update at first run

void readRSSI() {

  rssiRawValue = analogRead(RSSI_PIN);
  #ifndef RSSI_RAWVAL
    rssiRawValue = ((long)rssiRawValue - RSSI_0P) * 100 / (RSSI_100P - RSSI_0P);
    if (rssiRawValue < 0) {
      rssiRawValue = 0;
    }
    if (rssiRawValue > 100) {
      rssiRawValue = 100;
    }
  #endif
}

#endif  // #define _AQ_OSD_MAX7456_RSSI_H_
