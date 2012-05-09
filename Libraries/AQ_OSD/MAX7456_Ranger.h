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

#ifndef _AQ_OSD_MAX7456_RANGER_H_
#define _AQ_OSD_MAX7456_RANGER_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Range Finder /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//  0123456789012345678901234567
//  xx>>>>><<<<<<xx>>>>>><<<<<xx
//  xx12345 12345xx12345 12345xx
byte needClear = 0;

// Special characters
// 9b  9c  9d  9e  9f
// |.. ||. ||| .|| ..|
void fillBarGraph(char *buf, byte bufsize, byte bars, byte right) {

  for (byte i=0; i<bufsize; i++) {
    byte pos = right?(bufsize-1-i):i;
    if (bars - 2 > i*3) {
      buf[pos]=0x9d;
    }
    else if (bars - 1 > i*3) {
      buf[pos]=right?0x9e:0x9c;
    }
    else if (bars > i*3) {
      buf[pos]=right?0x9f:0x9b;
    }
    else {
      buf[pos]=0x20;
    }
  }
}

void displayRanger() {
  byte writeit=1;
  char buf[28];
  memset(buf,' ',28);
  if (isOnRangerRange(rangeFinderRange[LEFT_RANGE_FINDER_INDEX])) {
    byte range = (int)constrain(rangeFinderRange[LEFT_RANGE_FINDER_INDEX]*10,0,99);
    buf[0] = '0' + (range / 10);
    buf[1] = '0' + (range % 10);
    if (range>45) {
      range=0;
    } else {
      range = (45-range)/3;
    }
    fillBarGraph(buf+2, 5, range,0);
    writeit=1;
    needClear|=1;
  } else {
    if (needClear&1) {
      needClear&=~1;
      writeit=1;
    }
  }
  if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
    byte range = (int)constrain(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]*10,0,99);
    buf[13]='0'+(range/10);
    buf[14]='0'+(range%10);
    if (range>45) {
      range=0;
    } else {
      range = (45-range)/3;
    }
    fillBarGraph(buf+8, 5, range, 0);
    fillBarGraph(buf+15, 5, range, 1);    
    writeit=1;
    needClear|=2;
  } else {
    if (needClear&2) {
      needClear&=~2;
      writeit=1;
    }
  }
  if (isOnRangerRange(rangeFinderRange[RIGHT_RANGE_FINDER_INDEX])) {
    byte range = (int)constrain(rangeFinderRange[RIGHT_RANGE_FINDER_INDEX]*10,0,99);
    buf[26]= '0' + (range / 10);
    buf[27]= '0' + (range % 10);
    if (range>45) {
      range=0;
    } else {
      range = (45-range)/3;
    }
    fillBarGraph(buf+21, 5, range, 1);
    writeit=1;
    needClear|=4;
  } else {
    if (needClear&4) {
      needClear&=~4;
      writeit=1;
    }
  }
  if (writeit) {
    writeChars(buf, 28, 0, NOTIFY_ROW, 1);
  }
}

#endif  // #define _AQ_OSD_MAX7456_RSSI_H_
