#include <TinyGPS.h>
#include <avr\pgmspace.h>

/* This sample code demonstrates the basic use of a TinyGPS object.
   Typically, you would feed it characters from a serial GPS device, but 
   this example uses static strings for simplicity.
*/
prog_char str1[] PROGMEM = "$GPRMC,201547.000,A,3014.5527,N,09749.5808,W,0.24,163.05,040109,,*1A";
prog_char str2[] PROGMEM = "$GPGGA,201548.000,3014.5529,N,09749.5808,W,1,07,1.5,225.6,M,-22.5,M,18.8,0000*78";
prog_char str3[] PROGMEM = "$GPRMC,201548.000,A,3014.5529,N,09749.5808,W,0.17,53.25,040109,,*2B";
prog_char str4[] PROGMEM = "$GPGGA,201549.000,3014.5533,N,09749.5812,W,1,07,1.5,223.5,M,-22.5,M,18.8,0000*7C";
prog_char *teststrs[4] = {str1, str2, str3, str4};

void sendstring(TinyGPS &gps, const PROGMEM char *str);
void gpsdump(TinyGPS &gps);

void setup()
{
  Serial.begin(115200);
  
  Serial.print("Testing TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); Serial.println(sizeof(TinyGPS));
  Serial.println();

  for (int i=0; i<4; ++i)
  {
    TinyGPS test_gps;
    Serial.print("Test string #"); Serial.println(i+1);
    Serial.println("--------------");
    sendstring(test_gps, teststrs[i]);
    gpsdump(test_gps);
    Serial.println();
  }
}

void loop()
{
}

void printFloat(double number, int digits=5)
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

void sendstring(TinyGPS &gps, const PROGMEM char *str)
{
  while (true)
  {
    char c = pgm_read_byte_near(str++);
    if (!c) break;
    Serial.print(c);
    gps.encode(c);
  }
  Serial.println();
  gps.encode('\r');
  gps.encode('\n');
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
  Serial.print("Lat/Long(10^-5 deg): "); Serial.print(lat); Serial.print(", "); Serial.print(lon); 
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); printFloat(flat); Serial.print(", "); printFloat(flon);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); Serial.print(date); Serial.print(" Time(hhmmsscc): "); Serial.print(time);
  Serial.print(" Fix age: "); Serial.print(age); Serial.println("ms.");

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); Serial.print(static_cast<int>(month)); Serial.print("/"); Serial.print(static_cast<int>(day)); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(static_cast<int>(hour)); Serial.print(":"); Serial.print(static_cast<int>(minute)); Serial.print(":"); Serial.print(static_cast<int>(second)); Serial.print("."); Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  Serial.print(age); Serial.println("ms.");
  
  Serial.print("Alt(cm): "); Serial.print(gps.altitude()); Serial.print(" Course(10^-2 deg): "); Serial.print(gps.course()); Serial.print(" Speed(10^-2 knots): "); Serial.println(gps.speed());
  Serial.print("Alt(float): "); printFloat(gps.f_altitude(), 2); Serial.print(" Course(float): "); printFloat(gps.f_course(), 2); Serial.println();
  Serial.print("Speed (knots): "); printFloat(gps.f_speed_knots(), 2); Serial.print(" (mph): ");  printFloat(gps.f_speed_mph(), 2);
  Serial.print(" (mps): "); printFloat(gps.f_speed_mps(), 2); Serial.print(" (kmph): "); printFloat(gps.f_speed_kmph(), 2); Serial.println();
  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); Serial.print(chars); Serial.print(" sentences: "); Serial.print(sentences); Serial.print(" failed checksum: "); Serial.println(failed);
}
  
