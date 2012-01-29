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
*/

#ifndef TinyGPS_h
#define TinyGPS_h

#include "Arduino.h"

#define _GPS_VERSION 10 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001
//#define _GPS_NO_STATS

class TinyGPS
{
  public:
    TinyGPS();
    bool encode(char c); // process one character received from GPS
    TinyGPS &operator << (char c) {encode(c); return *this;}
    
    // lat/long in hundred thousandths of a degree and age of fix in milliseconds
    inline void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0)
    {
      if (latitude) *latitude = _latitude;
      if (longitude) *longitude = _longitude;
      if (fix_age) *fix_age = _last_position_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_position_fix;
    }

    // date as ddmmyy, time as hhmmsscc, and age in milliseconds
    inline void get_datetime(unsigned long *date, unsigned long *time, unsigned long *fix_age = 0)
    {
      if (date) *date = _date;
      if (time) *time = _time;
      if (fix_age) *fix_age = _last_time_fix == GPS_INVALID_FIX_TIME ? 
        GPS_INVALID_AGE : millis() - _last_time_fix;
    }

    // signed altitude in centimeters (from GPGGA sentence)
    inline long altitude() { return _altitude; }

    // course in last full GPRMC sentence in 100th of a degree
    inline unsigned long course() { return _course; }
    
    // speed in last full GPRMC sentence in 100ths of a knot
    unsigned long speed() { return _speed; }

#ifndef _GPS_NO_STATS
    void stats(unsigned long *chars, unsigned short *good_sentences, unsigned short *failed_cs);
#endif

    inline void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = 0)
    {
      long lat, lon;
      get_position(&lat, &lon, fix_age);
      *latitude = lat / 100000.0;
      *longitude = lon / 100000.0;
    }

    inline void crack_datetime(int *year, byte *month, byte *day, 
      byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0)
    {
      unsigned long date, time;
      get_datetime(&date, &time, fix_age);
      if (year) 
      {
        *year = date % 100;
        *year += *year > 80 ? 1900 : 2000;
      }
      if (month) *month = (date / 100) % 100;
      if (day) *day = date / 10000;
      if (hour) *hour = time / 1000000;
      if (minute) *minute = (time / 10000) % 100;
      if (second) *second = (time / 100) % 100;
      if (hundredths) *hundredths = time % 100;
    }

    inline float f_altitude()    { return altitude() / 100.0; }
    inline float f_course()      { return course() / 100.0; }
    inline float f_speed_knots() { return speed() / 100.0; }
    inline float f_speed_mph()   { return _GPS_MPH_PER_KNOT * f_speed_knots(); }
    inline float f_speed_mps()   { return _GPS_MPS_PER_KNOT * f_speed_knots(); }
    inline float f_speed_kmph()  { return _GPS_KMPH_PER_KNOT * f_speed_knots(); }

    static int library_version() { return _GPS_VERSION; }

    enum {GPS_INVALID_AGE = 0xFFFFFFFF, GPS_INVALID_ANGLE = 999999999, GPS_INVALID_ALTITUDE = 999999999, GPS_INVALID_DATE = 0,
      GPS_INVALID_TIME = 0xFFFFFFFF, GPS_INVALID_SPEED = 999999999, GPS_INVALID_FIX_TIME = 0xFFFFFFFF};


    static float distance_between (float lat1, float long1, float lat2, float long2);

private:
    enum {_GPS_SENTENCE_GPGGA, _GPS_SENTENCE_GPRMC, _GPS_SENTENCE_OTHER};
    
    // properties
    unsigned long _time, _new_time;
    unsigned long _date, _new_date;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    long _altitude, _new_altitude;
    unsigned long  _speed, _new_speed;
    unsigned long  _course, _new_course;

    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    byte _term_offset;
    bool _gps_data_good;

#ifndef _GPS_NO_STATS
    // statistics
    unsigned long _encoded_characters;
    unsigned short _good_sentences;
    unsigned short _failed_checksum;
    unsigned short _passed_checksum;
#endif

    // internal utilities
    int from_hex(char a);
    unsigned long parse_decimal();
    unsigned long parse_degrees();
    bool term_complete();
    bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
    long gpsatol(const char *str);
    int gpsstrcmp(const char *str1, const char *str2);
};

// Arduino 0012 workaround
#undef int
#undef char
#undef long
#undef byte
#undef float
#undef abs
#undef round 

#endif
