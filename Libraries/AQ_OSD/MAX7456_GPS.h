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

#ifndef _AQ_OSD_MAX7456_GPS_H_
#define _AQ_OSD_MAX7456_GPS_H_

//////////////////////////////////////////////////////////////////////////////
//////////////////////////// GPS Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show GPS information

void get_position(long *latitude, long *longitude, unsigned long *fixage);

byte foo=0;

long abslong(long x) {
  return (x<0)?-x:x;
}

void displayGPS() {

  {
    char buf[2]={176+foo*2,176+foo*2+1};
    writeChars(buf, 2, 0, GPS_HA_ROW, GPS_HA_COL);
    foo = (foo + 1) & 15;
  }

  {
    long lat,lon;
    get_position(&lat,&lon,NULL);
    if (lat == GPS_INVALID_ANGLE) {
      writeChars("Waiting for GPS fix", 20, 0, GPS_ROW, GPS_COL);
    } else {
      char buf[20];
      snprintf(buf,20,"%c%2ld.%05ld%c%3ld.%05ld",
               (lat>=0)?'N':'S',abs(lat)/100000L,abs(lat)%100000L,
               (lon>=0)?'E':'w',abs(lon)/100000L,abs(lon)%100000L);
      writeChars(buf, 20, 0, GPS_ROW, GPS_COL);
    }
  }
}

#endif  // #define _AQ_OSD_MAX7456_RSSI_H_
