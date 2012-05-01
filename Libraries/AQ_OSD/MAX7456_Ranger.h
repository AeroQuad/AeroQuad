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

// 0123456789012345678901234567890
//  xx>>>>><<<<<<<xx>>>>>><<<<<xx
byte needClear = 0;

void displayRanger() {
  byte writeit=1;
  char buf[28];
  memset(buf,' ',28);
  if (isOnRangerRange(rangeFinderRange[LEFT_RANGE_FINDER_INDEX])) {
    byte range = (int)constrain(rangeFinderRange[LEFT_RANGE_FINDER_INDEX]*10,0,99);
    buf[0]='0'+(range/10);
    buf[1]='0'+(range%10);
    buf[2]=range<40?'>':' ';
    buf[3]=range<20?'>':' ';
    buf[4]=range<10?'>':' ';
    buf[5]=range<07?'>':' ';
    buf[6]=range<05?'#':' ';
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
    buf[12]=buf[15]=range<35?'-':' ';
    buf[11]=buf[16]=range<25?'-':' ';
    buf[10]=buf[17]=range<20?'-':' ';
    buf[9]=buf[18]=range<15?'-':' ';
    buf[8]=buf[19]=range<10?'#':' ';
    buf[7]=buf[20]=range<05?'#':' ';
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
    buf[26]='0'+(range/10);
    buf[27]='0'+(range%10);
    buf[25]=range<40?'>':' ';
    buf[24]=range<20?'>':' ';
    buf[23]=range<10?'>':' ';
    buf[22]=range<07?'>':' ';
    buf[21]=range<05?'#':' ';
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
