/*
  AeroQuad v3.0 - March 2011
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

#include <Wire.h>             // Arduino IDE bug, needed because that the ITG3200 use Wire!
#include <Device_I2C.h>       // Arduino IDE bug, needed because that the ITG3200 use Wire!

#include <GlobalDefined.h>
#include <AQMath.h>

//Uncomment the following two lines when testing the mag on v2.1 shield
//#define SPARKFUN_9DOF_5883L
//#include <Magnetometer_HMC5883L.h>

//Uncomment the following line when testing the mag on v2.0 shield
#include <Magnetometer_HMC5843.h>

unsigned long timer;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Magnetometer library test (HMC58xx)");
  
  Wire.begin();
  initializeMagnetometer();
}

void loop() {
  
  if((millis() - timer) > 10) // 100Hz
  {
    timer = millis();
    measureMagnetometer(0.0,0.0);
    
    Serial.print("Roll: ");
    Serial.print(getMagnetometerRawData(XAXIS));
    Serial.print(" Pitch: ");
    Serial.print(getMagnetometerRawData(YAXIS));
    Serial.print(" Yaw: ");
    Serial.print(getMagnetometerRawData(ZAXIS));
    Serial.print(" Heading: ");
    Serial.print(getAbsoluteHeading());
    Serial.println();
  }
}
