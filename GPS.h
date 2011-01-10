/*
  AeroQuad v2.1.3 Beta - December 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

/**************************************************************/
/************************** GPS *******************************/
/**************************************************************/
// This is experimental, it is not yet functional

#ifdef GPS

TinyGPS gps;
NewSoftSerial nss(13, 3);

bool newdata;
unsigned long start;

void gpsdump(TinyGPS &gps);
bool feedgps();
void printFloat(double f, int digits = 2);

void printFloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
     SERIAL_PORT->print('-');
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
  SERIAL_PORT->print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    SERIAL_PORT->print(".");

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    SERIAL_PORT->print(toPrint);
    remainder -= toPrint;
  }
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  SERIAL_PORT->print("Lat/Long(10^-5 deg): "); SERIAL_PORT->print(lat); SERIAL_PORT->print(", "); SERIAL_PORT->print(lon);
  SERIAL_PORT->print(" Fix age: "); SERIAL_PORT->print(age); SERIAL_PORT->println("ms.");

  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  SERIAL_PORT->print("Lat/Long(float): "); printFloat(flat, 5); SERIAL_PORT->print(", "); printFloat(flon, 5);
  SERIAL_PORT->print(" Fix age: "); SERIAL_PORT->print(age); SERIAL_PORT->println("ms.");

  feedgps();

  gps.get_datetime(&date, &time, &age);
  SERIAL_PORT->print("Date(ddmmyy): "); SERIAL_PORT->print(date); SERIAL_PORT->print(" Time(hhmmsscc): "); SERIAL_PORT->print(time);
  SERIAL_PORT->print(" Fix age: "); SERIAL_PORT->print(age); SERIAL_PORT->println("ms.");

  feedgps();

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  SERIAL_PORT->print("Date: "); SERIAL_PORT->print(static_cast<int>(month)); SERIAL_PORT->print("/"); SERIAL_PORT->print(static_cast<int>(day)); SERIAL_PORT->print("/"); SERIAL_PORT->print(year);
  SERIAL_PORT->print("  Time: "); SERIAL_PORT->print(static_cast<int>(hour)); SERIAL_PORT->print(":"); SERIAL_PORT->print(static_cast<int>(minute)); SERIAL_PORT->print(":"); SERIAL_PORT->print(static_cast<int>(second)); SERIAL_PORT->print("."); SERIAL_PORT->print(static_cast<int>(hundredths));
  SERIAL_PORT->print("  Fix age: ");  SERIAL_PORT->print(age); SERIAL_PORT->println("ms.");

  feedgps();

  SERIAL_PORT->print("Alt(cm): "); SERIAL_PORT->print(gps.altitude()); SERIAL_PORT->print(" Course(10^-2 deg): "); SERIAL_PORT->print(gps.course()); SERIAL_PORT->print(" Speed(10^-2 knots): "); SERIAL_PORT->println(gps.speed());
  SERIAL_PORT->print("Alt(float): "); printFloat(gps.f_altitude()); SERIAL_PORT->print(" Course(float): "); printFloat(gps.f_course()); SERIAL_PORT->println();
  SERIAL_PORT->print("Speed(knots): "); printFloat(gps.f_speed_knots()); SERIAL_PORT->print(" (mph): ");  printFloat(gps.f_speed_mph());
  SERIAL_PORT->print(" (mps): "); printFloat(gps.f_speed_mps()); SERIAL_PORT->print(" (kmph): "); printFloat(gps.f_speed_kmph()); SERIAL_PORT->println();

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  SERIAL_PORT->print("Stats: characters: "); SERIAL_PORT->print(chars); SERIAL_PORT->print(" sentences: "); SERIAL_PORT->print(sentences); SERIAL_PORT->print(" failed checksum: "); SERIAL_PORT->println(failed);
}

bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

#endif


