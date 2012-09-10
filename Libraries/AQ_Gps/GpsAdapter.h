
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

#ifndef _AQ_GPS_ADAPTER_H_
#define _AQ_GPS_ADAPTER_H_

#include <GpsDataType.h>


#include <ublox.h>


#define MIN_NB_SATS_IN_USE 6

#define GPS2RAD (1.0/572957795.0)
#define RAD2DEG 57.2957795



GeodeticPosition currentPosition;

float cosLatitude = 0.7; // @ ~ 45 N/S, this will be adjusted to home loc 

boolean isGpsHaveANewPosition = false;

void initializeGps() {
    gpsdata.lat = GPS_INVALID_ANGLE;
    gpsdata.lon = GPS_INVALID_ANGLE;
    gpsdata.course = GPS_INVALID_ANGLE;
    gpsdata.speed = GPS_INVALID_SPEED;
    gpsdata.height = GPS_INVALID_ALTITUDE;
    gpsdata.accuracy = GPS_INVALID_ACCURACY;
    gpsdata.fixage = GPS_INVALID_AGE; 
    gpsdata.state = DETECTING;
    gpsdata.sats = 0;
    gpsdata.fixtime = 0xFFFFFFFF;
} gpsdata;
 
}

boolean readGps() {
  return isGpsHaveANewPosition;
}
  
boolean haveAGpsLock() {
  return gps->fix && gps->num_sats >= MIN_NB_SATS_IN_USE;
}

long getCourse() {
  return gps->ground_course;
}
unsigned long getGpsSpeed() {
  return gps->ground_speed;
}

unsigned long getGpsAltitude() {
  return gps->altitude;
}

void setProjectionLocation(struct GeodeticPosition pos) {

  cosLatitude = cos((float)pos.latitude * GPS2RAD);
}

float gpsRawDistance = 0.0;
float gpsBearing = 0;

void computeDistanceAndBearing(struct GeodeticPosition p1, struct GeodeticPosition p2) {

  const float x = (float)(p2.longitude - p1.longitude) * GPS2RAD * cosLatitude;
  const float y = (float)(p2.latitude - p1.latitude) * GPS2RAD;
  gpsRawDistance = sqrt(x*x+y*y);
  gpsBearing = (RAD2DEG * atan2(x,y));
}

float getDistanceMeter() {

  return (gpsRawDistance * 6371009);
}

float getDistanceFoot() {

  return (gpsRawDistance * 20903280);
}

#endif
