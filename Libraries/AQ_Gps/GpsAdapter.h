
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

#ifndef _AEROQUAD_TINY_GPS_ADAPTER_H_
#define _AEROQUAD_TINY_GPS_ADAPTER_H_

#include <AP_GPS.h>

#define MIN_NB_SATS_IN_USE 6

#define GPS2RAD (1/5729577.95)
#define RAD2DEG 57.2957795

enum {
  GPS_INVALID_AGE = 0xFFFFFFFF, 
  GPS_INVALID_ANGLE = 999999999, 
  GPS_INVALID_ALTITUDE = 999999999, 
  GPS_INVALID_DATE = 0,
  GPS_INVALID_TIME = 0xFFFFFFFF, 
  GPS_INVALID_SPEED = 999999999, 
  GPS_INVALID_FIX_TIME = 0xFFFFFFFF
};


struct GeodeticPosition {
  long latitude;
  long longitude;
  
  GeodeticPosition() {
    latitude = GPS_INVALID_ANGLE;
    longitude = GPS_INVALID_ANGLE;
  }
};
GeodeticPosition currentPosition;

byte nbSatelitesInUse = 0;
boolean isGpsHaveANewPosition = false;
GPS	    *gps;
AP_GPS_Auto GPS(&Serial1, &gps);

void initializeGps() {
 
  gps = &GPS;
  gps->init();
}

boolean readGps() {
  gps->update();
  if (gps->new_data) {
    isGpsHaveANewPosition = true;
	currentPosition.latitude = gps->latitude;
	currentPosition.longitude = gps->longitude;
	nbSatelitesInUse = gps->num_sats;
	gps->new_data = false;
  }
  return isGpsHaveANewPosition;
}
  
boolean haveAGpsLock() {
  return gps->fix && gps->num_sats >= MIN_NB_SATS_IN_USE;
}

long getCourse() {
  return gps->ground_course;
}
unsigned long getGpsSpeed() {
  return gps->ground_speed*1.852*10/36;
}

unsigned long getGpsAltitude() {
  return gps->altitude;
}
  


float gpsRawDistance = 0.0;
short gpsBearing = 0;

void computeDistanceAndBearing(long lat1, long lon1, long lat2, long lon2) {

  const float x = (float)(lon2-lon1) * GPS2RAD * cos((float)(lat1+lat2)/2*GPS2RAD);
  const float y = (float)(lat2-lat1) * GPS2RAD;
  gpsRawDistance = sqrt(x*x+y*y);
  gpsBearing = (short)(RAD2DEG * atan2(x,y));
}

float getDistanceMeter() {

  return (gpsRawDistance * 6371009);
}

float getDistanceFoot() {

  return (gpsRawDistance * 20903280);
}

#endif
