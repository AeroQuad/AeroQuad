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

#include <GpsAdapter.h>

byte osdGPSState=0;
#define GPS_DONAV 0x80 // display navigation info next time
#define GPS_NONAV 0x40 // nav info hidden (no fix or no target)

void displayGPS(struct GeodeticPosition pos, struct GeodeticPosition home, long speed, long course, float magheadingrad, unsigned int numsats) {

  short magheading = (int)(magheadingrad*RAD2DEG);
  if (osdGPSState & GPS_DONAV) {
    if ((home.latitude == GPS_INVALID_ANGLE) || (gpsData.state < GPS_FIX2D)) {
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
      computeDistanceAndBearing(pos, home);
      #ifdef USUnits
        const unsigned int distance = getDistanceFoot(); // dist to home in feet
	    if (distance<1000) {
          snprintf(buf,5,"%3df",(int)distance);
        }
        else if (distance<5280) {
          snprintf(buf,5,".%02dm", (int)(distance * 10 / 528));
        }
        else {
          snprintf(buf,5,"%d.%1dm", (int)(distance/5280), (int)(distance / 528 % 10));
        }
      #else //metric
        const unsigned int distance = getDistanceMeter(); // dist to home in meters
	    if (distance<1000) {
          snprintf(buf,5,"%3dm",(int)distance);
        }
        else {
          snprintf(buf,5,"%d.%1d\032", (int)(distance/1000), (int)(distance / 100 % 10));
        }
      #endif
      writeChars(buf, 4, 0, GPS_HA_ROW + 1, GPS_HA_COL - 1);

      short homearrow = gpsBearing - magheading; // direction of home vs. craft orientation

      homearrow = ((homearrow + 11) * 16 / 360 + 16) % 16; // map to the 16 direction arrow
      buf[0]=176 + homearrow * 2;
      buf[1]=buf[0]+1;
      writeChars(buf, 2, 0, GPS_HA_ROW, GPS_HA_COL);
    
      //  calculate course correction 
      short courseCorrection = (gpsBearing - course/100);
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
    if (gpsData.state==GPS_DETECTING) {
      char buf[29];
      snprintf(buf,29,"Detecting GPS");
      writeChars(buf, 28, 0, GPS_ROW, GPS_COL);
    } else if (gpsData.state==GPS_NOFIX) {
      char buf[29];
      snprintf(buf,29,"Waiting for GPS fix (%d/%d)",numsats,6);
      writeChars(buf, 28, 0, GPS_ROW, GPS_COL);
    } else {
      char buf[29];
#ifdef USUnits
      speed=speed*36/1609; // convert from cm/s to mph 
      snprintf(buf,29,"%d:%c%02ld.%06ld%c%03ld.%06ld%3ld\031",numsats,
               (pos.latitude>=0)?'N':'S',labs(pos.latitude)/10000000L,labs(pos.latitude)%10000000L/10,
               (pos.longitude>=0)?'E':'W',labs(pos.longitude)/10000000L,labs(pos.longitude)%10000000L/10,
	       speed);
#else
      speed=speed*36/1000; // convert from cm/s to kmh 
      snprintf(buf,29,"%d:%c%02ld.%06ld%c%03ld.%06ld%3ld\030",numsats,
               (pos.latitude>=0)?'N':'S',labs(pos.latitude)/10000000L,labs(pos.latitude)%10000000L/10,
               (pos.longitude>=0)?'E':'W',labs(pos.longitude)/10000000L,labs(pos.longitude)%10000000L/10,
	       speed);
#endif
      writeChars(buf, 28, 0, GPS_ROW, GPS_COL);
    }
  }
  osdGPSState^=GPS_DONAV;
}

#endif  // #define _AQ_OSD_MAX7456_GPS_H_
