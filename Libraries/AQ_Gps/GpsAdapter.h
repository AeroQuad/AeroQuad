
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

struct gpsData gpsData; // This is accessed by the parser functions directly !

// default to all protocols

#if (!defined(USEGPS_UBLOX) && ! defined(USEGPS_NMEA)) 
  #define USEGPS_UBLOX
  #define USEGPS_NMEA
#endif


#ifdef USEGPS_UBLOX
#include <ublox.h>
#endif

#ifdef USEGPS_NMEA
#include <nmea.h>
#endif

#define MIN_NB_SATS_IN_USE 6

#define GPS2RAD (1.0/572957795.0)
#define RAD2DEG 57.2957795

GeodeticPosition currentPosition;

float cosLatitude = 0.7; // @ ~ 45 N/S, this will be adjusted to home loc 

struct gpsType {
  const char *name;
  void (*init)();
  int  (*processData)(unsigned char);
};

const unsigned long gpsBaudRates[] = { 9600L, 19200L, 38400L, 57600L, 115200L};
const struct gpsType gpsTypes[] = {
#ifdef USEGPS_UBLOX
  { "UBlox", ubloxInit, ubloxProcessData },
#endif
#ifdef USEGPS_NMEA
  { "NMEA", nmeaInit, nmeaProcessData },
#endif
};

#define GPS_NUMBAUDRATES (sizeof(gpsBaudRates)/sizeof(gpsBaudRates[0]))
#define GPS_NUMTYPES     (sizeof(gpsTypes)/sizeof(gpsTypes[0]))

// Timeout for GPS
#define GPS_MAXIDLE_DETECTING 200 // 2 seconds at 100Hz 
#define GPS_MAXIDLE 1000          // 10 seconds at 100Hz 

void initializeGpsPlugin() {
  
  gpsData.lat = GPS_INVALID_ANGLE;
  gpsData.lon = GPS_INVALID_ANGLE;
  gpsData.course = GPS_INVALID_ANGLE;
  gpsData.speed = GPS_INVALID_SPEED;
  gpsData.height = GPS_INVALID_ALTITUDE;
  gpsData.accuracy = GPS_INVALID_ACCURACY;
  gpsData.fixage = GPS_INVALID_AGE; 
  gpsData.state = GPS_DETECTING;
  gpsData.sentences = 0;
  gpsData.sats = 0;
  gpsData.fixtime = 0xFFFFFFFF;
  GPS_SERIAL.begin(gpsBaudRates[gpsData.baudrate]);  
  gpsTypes[gpsData.type].init();
}

void initializeGps() {
    gpsData.baudrate = 0;
    gpsData.type = 0;
    initializeGpsPlugin();  
 }

void updateGps() {
  
  gpsData.idlecount++;
  
  while (GPS_SERIAL.available()) {
    unsigned char c = GPS_SERIAL.read();
    int ret=0;
    ret = gpsTypes[gpsData.type].processData(c);
    if (ret) {
      gpsData.idlecount=0;
      currentPosition.latitude=gpsData.lat;
      currentPosition.longitude=gpsData.lon;
      currentPosition.altitude=gpsData.height;
    }
  }
   
  if (gpsData.idlecount > ((gpsData.state == GPS_DETECTING) ? GPS_MAXIDLE_DETECTING : GPS_MAXIDLE)) {
    gpsData.idlecount=0; 
    if (gpsData.state == GPS_DETECTING) {
      // advance baudrate and/or type
      gpsData.baudrate++;
      if (gpsData.baudrate >= GPS_NUMBAUDRATES) {
	gpsData.baudrate = 0;
	gpsData.type++;
	if (gpsData.type >= GPS_NUMTYPES) {
	  gpsData.type = 0;
	}
      }
      GPS_SERIAL.begin(gpsBaudRates[gpsData.baudrate]);  
    }
    gpsData.state = GPS_DETECTING;
    initializeGpsPlugin();  
  }
}
  
boolean haveAGpsLock() {
  return (gpsData.state > GPS_NOFIX) && (gpsData.sats >= MIN_NB_SATS_IN_USE);
}

long getCourse() {
  return gpsData.course / 1000; // to whole degrees
}
unsigned long getGpsSpeed() {
  return gpsData.speed;
}

unsigned long getGpsFixTime() {
  return gpsData.fixtime;
}

unsigned long getGpsAltitude() {
  return gpsData.height;
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
