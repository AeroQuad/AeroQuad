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
  
  Adapted from latest TinyGPS from Kenny
*/

#ifndef _AQ_GPS_H_
#define _AQ_GPS_H_

#include "Arduino.h"

#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"


#define _GPS_VERSION 10 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001

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
  _GPS_SENTENCE_GPGGA, 
  _GPS_SENTENCE_GPRMC, 
  _GPS_SENTENCE_OTHER
};

// properties
unsigned long _time = GPS_INVALID_TIME;
unsigned long _new_time = GPS_INVALID_TIME;
unsigned long _date = GPS_INVALID_DATE;
unsigned long _new_date = GPS_INVALID_DATE;
long _latitude = GPS_INVALID_ANGLE;
long _new_latitude = GPS_INVALID_ANGLE;
long _longitude = GPS_INVALID_ANGLE; 
long _new_longitude = GPS_INVALID_ANGLE;
long _altitude = GPS_INVALID_ALTITUDE;
long _new_altitude = GPS_INVALID_ALTITUDE;
unsigned long _speed = GPS_INVALID_SPEED;
unsigned long _new_speed = GPS_INVALID_SPEED;
unsigned long _course = GPS_INVALID_ANGLE;
unsigned long _new_course = GPS_INVALID_ANGLE;

unsigned long _last_time_fix = GPS_INVALID_FIX_TIME; 
unsigned long _new_time_fix = GPS_INVALID_FIX_TIME;
unsigned long _last_position_fix = GPS_INVALID_FIX_TIME;
unsigned long _new_position_fix = GPS_INVALID_FIX_TIME;

// parsing state variables
byte _parity = 0;
boolean _is_checksum_term = false;
char _term[15];
byte _sentence_type = _GPS_SENTENCE_OTHER;
byte _term_number = 0;
byte _term_offset = 0;
boolean _gps_data_good = false;


//
// internal utilities
//
int from_hex(char a) 
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

bool gpsisdigit(char c) { 
  return c >= '0' && c <= '9'; 
}

long gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str)) {
    ret = 10 * ret + *str++ - '0';
  }
  return ret;
}

int gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2) {
    ++str1, ++str2;
  }
  return *str1;
}


unsigned long parse_decimal()
{
  char *p = _term;
  boolean isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * gpsatol(p);
  while (gpsisdigit(*p)) {
    ++p;
  }
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2])) {
        ret += p[2] - '0';
      }
    }
  }
  return isneg ? -ret : ret;
}

unsigned long parse_degrees()
{
  char *p;
  unsigned long left = gpsatol(_term);
  unsigned long tenk_minutes = (left % 100UL) * 10000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 1000;
    while (gpsisdigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 100000 + tenk_minutes / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
boolean term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * from_hex(_term[0]) + from_hex(_term[1]);
    if (checksum == _parity)
    {
      if (_gps_data_good)
      {
        _last_time_fix = _new_time_fix;
        _last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _GPS_SENTENCE_GPRMC:
          _time      = _new_time;
          _date      = _new_date;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
          break;
        case _GPS_SENTENCE_GPGGA:
          _altitude  = _new_altitude;
          _time      = _new_time;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          break;
        }

        return true;
      }
    }

    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM)) {
      _sentence_type = _GPS_SENTENCE_GPRMC;
    }
    else if (!gpsstrcmp(_term, _GPGGA_TERM)) {
      _sentence_type = _GPS_SENTENCE_GPGGA;
    }
    else {
      _sentence_type = _GPS_SENTENCE_OTHER;
    }
    
	return false;
  }

  if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
    switch((_sentence_type == _GPS_SENTENCE_GPGGA ? 200 : 100) + _term_number)
    {
      case 101: // Time in both sentences
      case 201:
        _new_time = parse_decimal();
        _new_time_fix = millis();
        break;
      case 102: // GPRMC validity
        _gps_data_good = _term[0] == 'A';
        break;
      case 103: // Latitude
      case 202:
        _new_latitude = parse_degrees();
        _new_position_fix = millis();
        break;
      case 104: // N/S
      case 203:
        if (_term[0] == 'S') {
          _new_latitude = -_new_latitude;
		}
        break;
      case 105: // Longitude
      case 204:
        _new_longitude = parse_degrees();
        break;
      case 106: // E/W
      case 205:
        if (_term[0] == 'W') {
          _new_longitude = -_new_longitude;
		}
        break;
      case 107: // Speed (GPRMC)
        _new_speed = parse_decimal();
        break;
      case 108: // Course (GPRMC)
        _new_course = parse_decimal();
        break;
      case 109: // Date (GPRMC)
        _new_date = gpsatol(_term);
        break;
      case 206: // Fix data (GPGGA)
        _gps_data_good = _term[0] > '0';
        break;
      case 209: // Altitude (GPGGA)
        _new_altitude = parse_decimal();
        break;
    }
  }

  return false;
}



boolean encode(char c)
{
  boolean valid_sentence = false;

  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term)) {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _GPS_SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1) {
    _term[_term_offset++] = c;
  }
  if (!_is_checksum_term) {
    _parity ^= c;
  }

  return valid_sentence;
}

// lat/long in hundred thousandths of a degree and age of fix in milliseconds
void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
{
  if (latitude) {
    *latitude = _latitude;
  }
  if (longitude) {
    *longitude = _longitude;
  }	
  if (fix_age) {
    *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - _last_position_fix;
  }
}

// date as ddmmyy, time as hhmmsscc, and age in milliseconds
inline void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
{
  if (date) {
    *date = _date;
  }
  if (time) {
    *time = _time;
  }
  if (fix_age) {
    *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - _last_time_fix;
  }
}

// signed altitude in centimeters (from GPGGA sentence)
long altitude() { 
  return _altitude; 
}

// course in last full GPRMC sentence in 100th of a degree
unsigned long course() { 
  return _course; 
}
    
// speed in last full GPRMC sentence in 100ths of a knot
unsigned long speed() {
  return _speed; 
}

void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0)
{
  long lat, lon;
  get_position(&lat, &lon, fix_age);
  *latitude = lat / 100000.0;
  *longitude = lon / 100000.0;
}

inline void crack_datetime(int *year, 
                           byte *month, 
						   byte *day, 
                           byte *hour, 
						   byte *minute, 
						   byte *second, 
						   byte *hundredths = 0, 
						   unsigned long *fix_age = 0)
{
  unsigned long date, time;
  get_datetime(&date, &time, fix_age);
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

float f_altitude() { 
  return altitude() / 100.0; 
}

float f_course() { 
  return course() / 100.0; 
}

float f_speed_knots() {
  return speed() / 100.0; 
}

float f_speed_mph() {
  return _GPS_MPH_PER_KNOT * f_speed_knots(); 
}

float f_speed_mps() { 
  return _GPS_MPS_PER_KNOT * f_speed_knots(); 
}

float f_speed_kmph() {
  return _GPS_KMPH_PER_KNOT * f_speed_knots(); 
}

int library_version() {
  return _GPS_VERSION; 
}

float distance_between (float lat1, float long1, float lat2, float long2) 
{
  // returns distance in meters between two positions, both specified 
  // as signed decimal-degrees latitude and longitude. Uses great-circle 
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  float delta = radians(long1-long2);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  return delta * 6372795; 
}

#endif





