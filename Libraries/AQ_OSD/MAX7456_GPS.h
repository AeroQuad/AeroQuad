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

byte osdGPSState=0;
#define GPS_DONAV 0x80 // display navigation info next time
#define GPS_NOFIX 0x40 // no fix displayed
#define GPS_NONAV 0x20 // nav info hidden (no fix or no target)

void displayGPS(long lat, long lon, long hlat, long hlon, long speed, long course, short magheading) {

  if (osdGPSState & GPS_DONAV) {
    if ((hlat==GPS_INVALID_ANGLE) || (lat==GPS_INVALID_ANGLE)) {
      if (!(osdGPSState&GPS_NONAV)) {
        // clear info
        writeChars(NULL, 2, 0, GPS_HA_ROW, GPS_HA_COL);
        writeChars(NULL, 4, 0, GPS_HA_ROW + 1, GPS_HA_COL - 1);
        writeChars(NULL, 4, 0, GPS_HA_ROW + 2, GPS_HA_COL - 1);
        osdGPSState|=GPS_NONAV;
      }
    }
    else {
      char buf[5];
      // update 'home arrow' and distance

      // Calculate direction and distance to home using "equirectangular projection"
      #define GPS2RAD (1/5729577.95)
      #define RAD2DEG 57.2957795
      const float x = (float)(hlon-lon) * GPS2RAD * cos((float)(lat+hlat)/2*GPS2RAD);
      const float y = (float)(hlat-lat) * GPS2RAD;
      const short distance = (sqrt(x*x+y*y) * 6371009); // dist to home in meters
      short bearing = (short)(RAD2DEG * atan2(x,y));    // bearing to 'home' in degrees -180 - 180

      short homearrow = bearing - magheading; // direction of home vs. craft orientation

      homearrow = ((homearrow + 11) * 16 / 360 + 16) % 16; // map to the 16 direction arrow
      buf[0]=176 + homearrow * 2;
      buf[1]=buf[0]+1;
      writeChars(buf, 2, 0, GPS_HA_ROW, GPS_HA_COL);
      if (distance<1000) {
        snprintf(buf,5,"%3dm",distance);
      }
      else {
        snprintf(buf,5,"%d.%1dk", distance/1000, distance / 100 % 10);
      }
      writeChars(buf, 4, 0, GPS_HA_ROW + 1, GPS_HA_COL - 1);
    
      //  calculate course correction 
      short courseCorrection = (bearing - course/100);
      // normalize to -180 - 180 
      if (courseCorrection>180) courseCorrection-=360;
      if (courseCorrection<-180) courseCorrection+=360;
      
      snprintf(buf,5,"%c%d",courseCorrection>0?'R':'L', abs(courseCorrection));
      writeChars(buf, 4, 0, GPS_HA_ROW + 2, GPS_HA_COL - 1);
    
      osdGPSState&=~GPS_NONAV;
    }
  }
  else {
    // update position and speed
    if (lat == GPS_INVALID_ANGLE) {
      if (!(osdGPSState&GPS_NOFIX)) {
        writeChars("Waiting for GPS fix", 28, 0, GPS_ROW, GPS_COL);
        osdGPSState|=GPS_NOFIX;
      }
    } else {
      char buf[29];
      speed=speed*36/1000; // convert from cm/s to kmh 
      snprintf(buf,29,"%c%02ld.%05ld %c%03ld.%05ld %3ld",
               (lat>=0)?'N':'S',abs(lat)/100000L,abs(lat)%100000L,
               (lon>=0)?'E':'W',abs(lon)/100000L,abs(lon)%100000L,
	       speed);
      writeChars(buf, 28, 0, GPS_ROW, GPS_COL);
      osdGPSState&=~GPS_NOFIX;
    }
  }
  osdGPSState^=GPS_DONAV;
}

#endif  // #define _AQ_OSD_MAX7456_GPS_H_
