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

#ifndef _SBUSRSSIReader_h
#define _SBUSRSSIReader_h

#if defined ReceiverSBUS

#define RSSI_WARN    50     // show alarm at %

long sbusFrameCountLast = 0;
short rssiRawValue = 0; // forces update at first run
float rssiTemp = 0;


void readRSSI() {
  // need to detect what rate sBUS is running at - 7ms (~143Hz) or 14ms (~71Hz)
  // if 143Hz, difference between current and last frame count will be 2 or 3
  // if difference is 1, data rate is 71Hz
  if (sbusRate == 0) {
    if (sbusFrameCountLast != 0) {
      if ((sbusFrameCount > 1) && (sbusFrameCount < 4)) {
        sbusRate = 143;
      } else if (sbusFrameCount <= 1) {
        sbusRate = 71;
      }
    } else {
      sbusFrameCountLast = sbusFrameCount;
      sbusFrameCount = 0;
    }
  } 
  rssiTemp = (1 - ((float)sbusFailSafeCount / sbusRate)) * 100;
  rssiRawValue = rssiTemp;
}



#endif
#endif
