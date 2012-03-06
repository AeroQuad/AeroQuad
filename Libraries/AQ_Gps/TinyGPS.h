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

//#define _GPS_VERSION 10 // software version of this library
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
  GPS_SENTENCE_GPGGA, 
  GPS_SENTENCE_GPRMC, 
  GPS_SENTENCE_OTHER
};

// properties
unsigned long _GPS_time = GPS_INVALID_TIME;
unsigned long _GPS_new_time = GPS_INVALID_TIME;
unsigned long _GPS_date = GPS_INVALID_DATE;
unsigned long _GPS_new_date = GPS_INVALID_DATE;
long _GPS_latitude = GPS_INVALID_ANGLE;
long _GPS_new_latitude = GPS_INVALID_ANGLE;
long _GPS_longitude = GPS_INVALID_ANGLE; 
long _GPS_new_longitude = GPS_INVALID_ANGLE;
long _GPS_gpsAltitude = GPS_INVALID_ALTITUDE;
long _GPS_new_GPS_altitude = GPS_INVALID_ALTITUDE;
unsigned long _GPS_gpsSpeed = GPS_INVALID_SPEED;
unsigned long _GPS_new_speed = GPS_INVALID_SPEED;
unsigned long _GPS_course = GPS_INVALID_ANGLE;
unsigned long _GPS_new_course = GPS_INVALID_ANGLE;

unsigned long _GPS_last_time_fix = GPS_INVALID_FIX_TIME; 
unsigned long _GPS_new_time_fix = GPS_INVALID_FIX_TIME;
unsigned long _GPS_last_position_fix = GPS_INVALID_FIX_TIME;
unsigned long _GPS_new_position_fix = GPS_INVALID_FIX_TIME;

unsigned int _GPS_satelites_in_use = 0;
unsigned int _GPS_new_satelites_in_use = 0;

#ifndef _GPS_NO_STATS		
	unsigned long _GPS_encoded_characters = 0;
	unsigned short _GPS_good_sentences = 0;
	unsigned short _GPS_failed_checksum = 0;
    unsigned short _GPS_passed_checksum;
#endif

// parsing state variables
byte _GPS_parity = 0;
boolean _GPS_is_checksum_term = false;
char _GPS_term[15];
byte _GPS_sentence_type = GPS_SENTENCE_OTHER;
byte _GPS_term_number = 0;
byte _GPS_term_offset = 0;
boolean _GPS_gps_data_good = false;


//
// internal utilities
//
int GPS_from_hex(char a) 
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

bool GPS_gpsisdigit(char c) { 
  return c >= '0' && c <= '9'; 
}

long GPS_gpsatol(const char *str)
{
  long ret = 0;
  while (GPS_gpsisdigit(*str)) {
    ret = 10 * ret + *str++ - '0';
  }
  return ret;
}

int GPS_gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2) {
    ++str1, ++str2;
  }
  return *str1;
}


unsigned long GPS_parse_decimal()
{
  char *p = _GPS_term;
  boolean isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * GPS_gpsatol(p);
  while (GPS_gpsisdigit(*p)) {
    ++p;
  }
  if (*p == '.')
  {
    if (GPS_gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (GPS_gpsisdigit(p[2])) {
        ret += p[2] - '0';
      }
    }
  }
  return isneg ? -ret : ret;
}

unsigned long GPS_parse_degrees()
{
  char *p;
  unsigned long left = GPS_gpsatol(_GPS_term);
  unsigned long tenk_minutes = (left % 100UL) * 10000UL;
  for (p=_GPS_term; GPS_gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 1000;
    while (GPS_gpsisdigit(*++p))
    {
      tenk_minutes += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left / 100) * 10000000 + tenk_minutes * 100 / 6;
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
boolean GPS_term_complete()
{
  if (_GPS_is_checksum_term)
  {
    byte checksum = 16 * GPS_from_hex(_GPS_term[0]) + GPS_from_hex(_GPS_term[1]);
    if (checksum == _GPS_parity)
    {
      if (_GPS_gps_data_good)
      {
        _GPS_last_time_fix = _GPS_new_time_fix;
        _GPS_last_position_fix = _GPS_new_position_fix;

        switch(_GPS_sentence_type)
        {
        case GPS_SENTENCE_GPRMC:
          _GPS_time      = _GPS_new_time;
          _GPS_date      = _GPS_new_date;
          _GPS_latitude  = _GPS_new_latitude;
          _GPS_longitude = _GPS_new_longitude;
		  _GPS_satelites_in_use = _GPS_new_satelites_in_use;
          _GPS_gpsSpeed  = _GPS_new_speed;
          _GPS_course    = _GPS_new_course;
          break;
        case GPS_SENTENCE_GPGGA:
          _GPS_gpsAltitude = _GPS_new_GPS_altitude;
          _GPS_time        = _GPS_new_time;
          _GPS_latitude    = _GPS_new_latitude;
          _GPS_longitude   = _GPS_new_longitude;
          break;
        }

        return true;
      }
    }

    return false;
  }

  // the first term determines the sentence type
  if (_GPS_term_number == 0)
  {
    if (!GPS_gpsstrcmp(_GPS_term, _GPRMC_TERM)) {
      _GPS_sentence_type = GPS_SENTENCE_GPRMC;
    }
    else if (!GPS_gpsstrcmp(_GPS_term, _GPGGA_TERM)) {
      _GPS_sentence_type = GPS_SENTENCE_GPGGA;
    }
    else {
      _GPS_sentence_type = GPS_SENTENCE_OTHER;
    }
    
	return false;
  }

  if (_GPS_sentence_type != GPS_SENTENCE_OTHER && _GPS_term[0]) {
    switch((_GPS_sentence_type == GPS_SENTENCE_GPGGA ? 200 : 100) + _GPS_term_number)
    {
      case 101: // Time in both sentences
      case 201:
        _GPS_new_time = GPS_parse_decimal();
        _GPS_new_time_fix = millis();
        break;
      case 102: // GPRMC validity
        _GPS_gps_data_good = _GPS_term[0] == 'A';
        break;
      case 103: // Latitude
      case 202:
        _GPS_new_latitude = GPS_parse_degrees();
        _GPS_new_position_fix = millis();
        break;
      case 104: // N/S
      case 203:
        if (_GPS_term[0] == 'S') {
          _GPS_new_latitude = -_GPS_new_latitude;
		}
        break;
      case 105: // Longitude
      case 204:
        _GPS_new_longitude = GPS_parse_degrees();
        break;
      case 106: // E/W
      case 205:
        if (_GPS_term[0] == 'W') {
          _GPS_new_longitude = -_GPS_new_longitude;
		}
        break;
      case 107: // Speed (GPRMC)
        _GPS_new_speed = GPS_parse_decimal();
        break;
      case 108: // Course (GPRMC)
        _GPS_new_course = GPS_parse_decimal();
        break;
      case 109: // Date (GPRMC)
        _GPS_new_date = GPS_gpsatol(_GPS_term);
        break;
      case 206: // Fix data (GPGGA)
        _GPS_gps_data_good = _GPS_term[0] > '0';
        break;
      case 207: //Number of satelites in use
        _GPS_new_satelites_in_use = GPS_gpsatol(_GPS_term);
      case 209: // Altitude (GPGGA)
        _GPS_new_GPS_altitude = GPS_parse_decimal();
        break;
    }
  }

  return false;
}

boolean GPS_encode(char c)
{
  boolean valid_sentence = false;

  #ifndef _GPS_NO_STATS
  ++_GPS_encoded_characters;
  #endif
  
  switch(c)
  {
  case ',': // term terminators
    _GPS_parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_GPS_term_offset < sizeof(_GPS_term)) {
      _GPS_term[_GPS_term_offset] = 0;
      valid_sentence = GPS_term_complete();
    }
    ++_GPS_term_number;
    _GPS_term_offset = 0;
    _GPS_is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _GPS_term_number = _GPS_term_offset = 0;
    _GPS_parity = 0;
    _GPS_sentence_type = GPS_SENTENCE_OTHER;
    _GPS_is_checksum_term = false;
    _GPS_gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_GPS_term_offset < sizeof(_GPS_term) - 1) {
    _GPS_term[_GPS_term_offset++] = c;
  }
  if (!_GPS_is_checksum_term) {
    _GPS_parity ^= c;
  }

  return valid_sentence;
}

// lat/long in hundred thousandths of a degree and age of fix in milliseconds
void GPS_get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
{
  if (latitude) {
    *latitude = _GPS_latitude;
  }
  if (longitude) {
    *longitude = _GPS_longitude;
  }	
  if (fix_age) {
    *fix_age = _GPS_last_position_fix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - _GPS_last_position_fix;
  }
}

// date as ddmmyy, time as hhmmsscc, and age in milliseconds
inline void GPS_get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
{
  if (date) {
    *date = _GPS_date;
  }
  if (time) {
    *time = _GPS_time;
  }
  if (fix_age) {
    *fix_age = _GPS_last_time_fix == GPS_INVALID_FIX_TIME ? GPS_INVALID_AGE : millis() - _GPS_last_time_fix;
  }
}

// signed GPS_altitude in centimeters (from GPGGA sentence)
inline long GPS_get_altitude() { 
  return _GPS_gpsAltitude; 
}

// course in last full GPRMC sentence in 100th of a degree
inline unsigned long GPS_get_course() { 
  return _GPS_course; 
}
    
// speed in last full GPRMC sentence in 100ths of a knot
inline unsigned long GPS_get_speed() {
  return _GPS_gpsSpeed; 
}

inline unsigned int GPS_get_satelites_in_use() {
  return _GPS_satelites_in_use;
}

void GPS_f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0)
{
  long lat, lon;
  GPS_get_position(&lat, &lon, fix_age);
  *latitude = lat / 10000000.0;
  *longitude = lon / 10000000.0;
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
  GPS_get_datetime(&date, &time, fix_age);
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
  return _GPS_MPH_PER_KNOT * GPS_f_speed_knots(); 
}

float GPS_f_speed_mps() { 
  return _GPS_MPS_PER_KNOT * GPS_f_speed_knots(); 
}

float GPS_f_speed_kmph() {
  return _GPS_KMPH_PER_KNOT * GPS_f_speed_knots(); 
}

float GPS_f_speed_cmps() {
  return GPS_get_speed()*_GPS_KMPH_PER_KNOT*10/36; 
}

#endif

/*long GPS_distance_between (long lat1, long long1, long lat2, long long2) 
{
	if(lat1 == 0 || long1 == 0) 
		return -1;
	if(lat2 == 0 || long2 == 0) 
		return -1;

	float dlat 		= (float)(lat1 - lat2);
	float dlong  	= ((float)(long1 - long2));
	return sqrt(sq(dlat) * 1.113195 + sq(dlong) * 0.649876) ;
}*/
