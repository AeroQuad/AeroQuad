
/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AQ_GPS_DATA_TYPE_H_
#define _AQ_GPS_DATA_TYPE_H_

enum {
  GPS_INVALID_ACCURACY = 0xFFFFFFFF, 
  GPS_INVALID_AGE = 0xFFFFFFFF, 
  GPS_INVALID_ANGLE = 0x7FFFFFFF, 
  GPS_INVALID_ALTITUDE = 2147483647,//999999999, 
  GPS_INVALID_DATE = 0,
  GPS_INVALID_TIME = 0xFFFFFFFF, 
  GPS_INVALID_SPEED = 999999999, 
  GPS_INVALID_FIX_TIME = 0xFFFFFFFF
};

enum { 
    GPS_DETECTING = 0, 
    GPS_NOFIX = 1,
    GPS_FIX2D = 2,
    GPS_FIX3D = 3,
    GPS_FIX3DD = 4 // differential fix 
};

enum { 
    GPS_UBLOX = 0,
    GPS_NMEA  = 1,
    GPS_MTK16 = 2
};


#define GPS_INVALID_POSITION {GPS_INVALID_ANGLE, GPS_INVALID_ANGLE, 0}

struct GeodeticPosition {
  long latitude;
  long longitude;
  long altitude;
};

struct gpsData {
    int32_t  lat,lon;  // position as degrees (*10E7)
    int32_t  course;   // degrees (*10E5)
    uint32_t speed;    // cm/s
    int32_t  height;   // mm (from ellipsoid)
    uint32_t accuracy; // mm
    uint32_t fixage;   // fix 
    uint32_t fixtime;  // fix 
    uint32_t sentences; // sentences/packets processed from gps (just statistics)
    uint8_t  state;    // gps state
    uint8_t  sats;     // number of satellites active
    uint8_t  baudrate; // current baudrate (index) - used by autodetection
    uint8_t  type;     // current type - used by autodetection
    uint32_t idlecount; // how many times gpsUpdate has been called without getting a valid message
};

struct gpsConfigEntry {
  const unsigned char *data;
  const unsigned char len;
};

#endif
