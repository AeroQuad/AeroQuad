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
#include <Platform_CHR6DM.h>  // Arduino IDE bug, needed because that the CHR6DM use Wire!

#include <Axis.h>
#include <APM_ADC.h>
#include <AQMath.h>
#include <Gyroscope_APM.h>

unsigned long timer;
Gyroscope_APM gyro;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Gyroscope library test (APM)");
  
  initializeADC();
  gyro.calibrate();
}

void loop() {
  
  if((millis() - timer) > 10) // 100Hz
  {
    timer = millis();
    gyro.measure();
    
    Serial.print("Roll: ");
    Serial.print(degrees(gyro.getRadPerSec(ROLL)));
    Serial.print(" Pitch: ");
    Serial.print(degrees(gyro.getRadPerSec(PITCH)));
    Serial.print(" Yaw: ");
    Serial.print(degrees(gyro.getRadPerSec(YAW)));
    Serial.print(" Heading: ");
    Serial.print(degrees(gyro.getHeading()));
    Serial.println();
  }

}
