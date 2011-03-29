/*
  AeroQuad v2.3 - March 2011
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

#define ROLL 0
#define PITCH 1
#define YAW 2

////////////////////////////////////////////////////////////////////////
// GYROSCOPE CONFIG TEST SELECTION
////////////////////////////////////////////////////////////////////////
#define TEST_ITG3200
//#define TEST_GYROSCOPE_OILPAN
//#define TEST_GYROSCOPE_WII

////////////////////////////////////////////////////////////////////////
// GYRO ITG3200 TEST CONFIGURATION
////////////////////////////////////////////////////////////////////////
#if defined(TEST_ITG3200)
  #include <Wire.h> 
  #include <I2C.h>
  #include <AQMath.h>
  #include <Gyroscope_ITG3200.h>
  Gyroscope_ITG3200 tempGyroscope;
  Gyroscope *gyroscope = &tempGyroscope;
  
  void initPlatoformSpecific() {
    Wire.begin();  
    TWBR = 12;
    
    gyroscope->initialize();
  }
#endif

////////////////////////////////////////////////////////////////////////
// GYRO GYROSCOPE OILPAN TEST CONFIGURATION
////////////////////////////////////////////////////////////////////////
#if defined(TEST_GYROSCOPE_OILPAN)

  void initPlatoformSpecific() {
  }
#endif

////////////////////////////////////////////////////////////////////////
// GYRO GYROSCOPE WII TEST CONFIGURATION
////////////////////////////////////////////////////////////////////////
#if defined(TEST_GYROSCOPE_WII)

  void initPlatoformSpecific() {
  }
#endif

////////////////////////////////////////////////////////////////////////
// SPECIFIG GYROSCOPE INIT FUNCTION POINTER INITALIZATION
////////////////////////////////////////////////////////////////////////
void (*initPlatform)() = &initPlatoformSpecific;

////////////////////////////////////////////////////////////////////////
// MAIN SETUP FUNCTION
////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  
  // Init a specific gyro configuration
  initPlatform();
  
  // calibrate the gyro
  gyroscope->calibrate();
}

////////////////////////////////////////////////////////////////////////
// MAIN LOOP FUNCTION
////////////////////////////////////////////////////////////////////////
void loop() {
  gyroscope->measure();
  Serial.print("(");
  Serial.print(gyroscope->getRadPerSec(ROLL));
  Serial.print(",");
  Serial.print(gyroscope->getRadPerSec(PITCH));
  Serial.print(",");
  Serial.print(gyroscope->getRadPerSec(YAW));
  Serial.println(")");
}


