/*
  TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
  Based on work by and "distance_to" courtesy of Maarten Lamers.
  Copyright (C) 2008-2011 Mikal Hart
  All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Adapted from latest TinyGPS by Kenny9999
*/

#ifndef _AQ_GPS_H_
#define _AQ_GPS_H_

#include "Arduino.h"

#define GPRMC_TERM   "GPRMC"
#define GPGGA_TERM   "GPGGA"

//#define _GPS_VERSION 10 // software version of this library
#define GPS_MPH_PER_KNOT 1.15077945
#define GPS_MPS_PER_KNOT 0.51444444
#define GPS_KMPH_PER_KNOT 1.852
#define GPS_MILES_PER_METER 0.00062137112
#define GPS_KM_PER_METER 0.001

enum {
  GPS_INVALID_AGE = 0xFFFFFFFF, 
  GPS_INVALID_ANGLE = 999999999, 
  GPS_INVALID_ALTITUDE = 999999999, 
  GPS_INVALID_DATE = 0,
  GPS_INVALID_TIME = 0xFFFFFFFF, 
  GPS_INVALID_SPEED = 999999999, 
  GPS_INVALID_FIX_TIME = 0xFFFFFFFF
};

enum {
  GPS_SENTENCE_GPGGA, 
  GPS_SENTENCE_GPRMC, 
  GPS_SENTENCE_OTHER
};

// properties
unsigned long gpsTime = GPS_INVALID_TIME;
unsigned long gpsNewTime = GPS_INVALID_TIME;
unsigned long gpsDate = GPS_INVALID_DATE;
unsigned long gpsNewDate = GPS_INVALID_DATE;
long latitude = GPS_INVALID_ANGLE;
long newLatitude = GPS_INVALID_ANGLE;
long longitude = GPS_INVALID_ANGLE; 
long newLongitude = GPS_INVALID_ANGLE;
long gpsAltitude = GPS_INVALID_ALTITUDE;
long gpsNewAltitude = GPS_INVALID_ALTITUDE;
unsigned long gpsSpeed = GPS_INVALID_SPEED;
unsigned long gpsNewSpeed = GPS_INVALID_SPEED;
unsigned long gpsCourse = GPS_INVALID_ANGLE;
unsigned long gpsNewCourse = GPS_INVALID_ANGLE;

unsigned long gpsLastTimeFix = GPS_INVALID_FIX_TIME; 
unsigned long gpsNewTimeFix = GPS_INVALID_FIX_TIME;
unsigned long gpsLastPositionFix = GPS_INVALID_FIX_TIME;
unsigned long gpsNewPositionFix = GPS_INVALID_FIX_TIME;

unsigned int nbSatelitesInUse = 0;
unsigned int newNbSatelitesInUse = 0;

// parsing state variables
byte gpsParity = 0;
boolean isChecksumTerm = false;
char gpsTerm[15];
byte gpsSentenceType = GPS_SENTENCE_OTHER;
byte gpsTermNumber = 0;
byte gpsTermOffset = 0;
boolean isGpsDataGood = false;


//
// internal utilities
//
int fromHex(char a) 
{
  if (a >= 'A' && a <= 'F') {
    return a - 'A' + 10;
  }
  else if (a >= 'a' && a <= 'f') {
    return a - 'a' + 10;
  }
  else {
    return a - '0';
  }
}

bool isDigit(char c) { 
  return c >= '0' && c <= '9'; 
}

long atol(const char *str)
{
  long ret = 0;
  while (isDigit(*str)) {
    ret = 10 * ret + *str++ - '0';
  }
  return ret;
}

int strcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2) {
    ++str1, ++str2;
  }
  return *str1;
}


unsigned long parseDecimal()
{
  char *p = gpsTerm;
  boolean isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * atol(p);
  while (isDigit(*p)) {
    ++p;
  }
  if (*p == '.')
  {
    if (isDigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (isDigit(p[2])) {
        ret += p[2] - '0';
      }
    }
  }
  return isneg ? -ret : ret;
}

unsigned long parseDegrees()
{
  char *p;
  unsigned long left = atol(gpsTerm);
  unsigned long tenk_minutes = (left % 100UL) * 10000UL;
  for (p=gpsTerm; isDigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 1000;
    while (isDigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 10000000 + tenk_minutes * 100 / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
boolean gpsTermComplete()
{
  if (isChecksumTerm)
  {
    byte checksum = 16 * fromHex(gpsTerm[0]) + fromHex(gpsTerm[1]);
    if (checksum == gpsParity)
    {
      if (isGpsDataGood)
      {
        gpsLastTimeFix = gpsNewTimeFix;
        gpsLastPositionFix = gpsNewPositionFix;

        switch(gpsSentenceType)
        {
        case GPS_SENTENCE_GPRMC:
          gpsTime          = gpsNewTime;
          gpsDate          = gpsNewDate;
          latitude         = newLatitude;
          longitude        = newLongitude;
		  nbSatelitesInUse = newNbSatelitesInUse;
          gpsSpeed         = gpsNewSpeed;
          gpsCourse        = gpsNewCourse;
          break;
        case GPS_SENTENCE_GPGGA:
          gpsAltitude      = gpsNewAltitude;
          gpsTime          = gpsNewTime;
          latitude         = newLatitude;
          longitude        = newLongitude;
          break;
        }

        return true;
      }
    }

    return false;
  }

  // the first term determines the sentence type
  if (gpsTermNumber == 0)
  {
    if (!strcmp(gpsTerm, GPRMC_TERM)) {
      gpsSentenceType = GPS_SENTENCE_GPRMC;
    }
    else if (!strcmp(gpsTerm, GPGGA_TERM)) {
      gpsSentenceType = GPS_SENTENCE_GPGGA;
    }
    else {
      gpsSentenceType = GPS_SENTENCE_OTHER;
    }
    
	return false;
  }

  if (gpsSentenceType != GPS_SENTENCE_OTHER && gpsTerm[0]) {
    switch((gpsSentenceType == GPS_SENTENCE_GPGGA ? 200 : 100) + gpsTermNumber)
    {
      case 101: // Time in both sentences
      case 201:
        gpsNewTime = parseDecimal();
        gpsNewTimeFix = millis();
        break;
      case 102: // GPRMC validity
        isGpsDataGood = gpsTerm[0] == 'A';
        break;
      case 103: // Latitude
      case 202:
        newLatitude = parseDegrees();
        gpsNewPositionFix = millis();
        break;
      case 104: // N/S
      case 203:
        if (gpsTerm[0] == 'S') {
          newLatitude = -newLatitude;
		}
        break;
      case 105: // Longitude
      case 204:
        newLongitude = parseDegrees();
        break;
      case 106: // E/W
      case 205:
        if (gpsTerm[0] == 'W') {
          newLongitude = -newLongitude;
		}
        break;
      case 107: // Speed (GPRMC)
        gpsNewSpeed = parseDecimal();
        break;
      case 108: // Course (GPRMC)
        gpsNewCourse = parseDecimal();
        break;
      case 109: // Date (GPRMC)
        gpsNewDate = atol(gpsTerm);
        break;
      case 206: // Fix data (GPGGA)
        isGpsDataGood = gpsTerm[0] > '0';
        break;
      case 207: //Number of satelites in use
        newNbSatelitesInUse = atol(gpsTerm);
      case 209: // Altitude (GPGGA)
        gpsNewAltitude = parseDecimal();
        break;
    }
  }

  return false;
}

boolean decodeGpsChar(char c)
{
  boolean valid_sentence = false;

  switch(c)
  {
  case ',': // term terminators
    gpsParity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (gpsTermOffset < sizeof(gpsTerm)) {
      gpsTerm[gpsTermOffset] = 0;
      valid_sentence = gpsTermComplete();
    }
    ++gpsTermNumber;
    gpsTermOffset = 0;
    isChecksumTerm = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    gpsTermNumber = gpsTermOffset = 0;
    gpsParity = 0;
    gpsSentenceType = GPS_SENTENCE_OTHER;
    isChecksumTerm = false;
    isGpsDataGood = false;
    return valid_sentence;
  }

  // ordinary characters
  if (gpsTermOffset < sizeof(gpsTerm) - 1) {
    gpsTerm[gpsTermOffset++] = c;
  }
  if (!isChecksumTerm) {
    gpsParity ^= c;
  }

  return valid_sentence;
}

// lat/long in hundred thousandths of a degree and age of fix in milliseconds
/*void getPosition(long *lat, long *longi, unsigned long *fixAge = 0)
{
  if (lat) {
    *lat = latitude;
  }
  if (longi) {
    *longi = longitude;
  }	
  if (fixAge) {
    *fixAge = gpsLastPositionFix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - gpsLastPositionFix;
  }
}


// date as ddmmyy, time as hhmmsscc, and age in milliseconds
inline void getDateAndTime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
{
  if (date) {
    *date = gpsDate;
  }
  if (time) {
    *time = gpsTime;
  }
  if (fix_age) {
    *fix_age = gpsLastTimeFix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - gpsLastTimeFix;
  }
}

// signed GPS_altitude in centimeters (from GPGGA sentence)
inline long GPS_get_altitude() { 
  return gpsAltitude; 
}

// course in last full GPRMC sentence in 100th of a degree
inline unsigned long GPS_get_course() { 
  return gpsCourse; 
}
    
// speed in last full GPRMC sentence in 100ths of a knot
inline unsigned long GPS_get_speed() {
  return gpsSpeed; 
}

inline unsigned int GPS_get_satelites_in_use() {
  return nbSatelitesInUse;
}

void GPS_f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0)
{
  long lat, lon;
  getPosition(&lat, &lon, fix_age);
  *latitude = lat / 10000000.0;
  *longitude = lon / 10000000.0;
}



inline void extractDateAndTime(int *year, 
                           byte *month, 
						   byte *day, 
                           byte *hour, 
						   byte *minute, 
						   byte *second, 
						   byte *hundredths = 0, 
						   unsigned long *fix_age = 0)
{
  unsigned long date, time;
  getDateAndTime(&date, &time, fix_age);
  if (year) {
	*year = date % 100;
	*year += *year > 80 ? 1900 : 2000;
  }
  if (month) {
    *month = (date / 100) % 100;
  }
  if (day) {
    *day = date / 10000;
  }
  if (hour) {
    *hour = time / 1000000;
  }
  if (minute) {
    *minute = (time / 10000) % 100;
  }
  if (second) {
    *second = (time / 100) % 100;
  }
  if (hundredths) {
    *hundredths = time % 100;
  }
}

float GPS_f_altitude() { 
  return GPS_get_altitude() / 100.0; 
}

float GPS_f_course() { 
  return GPS_get_course() / 100.0; 
}

float GPS_f_speed_knots() {
  return GPS_get_speed() / 100.0; 
}

float GPS_f_speed_mph() {
  return GPS_MPH_PER_KNOT * GPS_f_speed_knots(); 
}

float GPS_f_speed_mps() { 
  return GPS_MPS_PER_KNOT * GPS_f_speed_knots(); 
}

float GPS_f_speed_kmph() {
  return GPS_KMPH_PER_KNOT * GPS_f_speed_knots(); 
}

float GPS_f_speed_cmps() {
  return GPS_get_speed()*GPS_KMPH_PER_KNOT*10/36; 
*/

#endif

/*long getDistanceBetween (long lat1, long long1, long lat2, long long2) 
{
	if(lat1 == 0 || long1 == 0) 
		return -1;
	if(lat2 == 0 || long2 == 0) 
		return -1;

	float dlat 		= (float)(lat1 - lat2);
	float dlong  	= ((float)(long1 - long2));
	return sqrt(sq(dlat) * 1.113195 + sq(dlong) * 0.649876) ;
}*/
