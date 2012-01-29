
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

#include "Arduino.h"
#include "TinyGPS.h"

#define GPS_SERIAL_BAUD_SPEED 38400  
#define GPS_PORT Serial1



void initializeGps() {
  GPS_PORT.begin(GPS_SERIAL_BAUD_SPEED);
}

boolean readGps()
{
  while (GPS_PORT.available())
  {
    if (encode(GPS_PORT.read()))
      return true;
  }
  return false;
}

boolean isGpsHaveALock() {
 return _last_position_fix != GPS_INVALID_FIX_TIME;
}




/*void printGpsFloat(double number, int digits = 0)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     Serial.print('-');
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}

void gpsdump()
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");
  
  readGps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printGpsFloat(flat, 5); Serial.print(", "); printGpsFloat(flon, 5);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  readGps();

  get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): "); Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  readGps();

  crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second)); Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");
  
  readGps();

  Serial.print("Alt(cm): "); Serial.print(altitude()); Serial.print(" Course(10^-2 deg): "); Serial.print(course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(speed());
  Serial.print("Alt(float): "); printGpsFloat(f_altitude()); Serial.print(" Course(float): "); printGpsFloat(f_course()); Serial.println();
  Serial.print("Speed(knots): "); printGpsFloat(f_speed_knots()); Serial.print(" (mph): ");  printGpsFloat(f_speed_mph());
  Serial.print(" (mps): "); printGpsFloat(f_speed_mps()); Serial.print(" (kmph): "); printGpsFloat(f_speed_kmph()); Serial.println();

  readGps();
}
*/  


#endif