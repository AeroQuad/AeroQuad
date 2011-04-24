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

#include <APM_ADC.h>          // Arduino IDE bug, needed because that the APM use it
#include <Platform_CHR6DM.h>  // Arduino IDE bug, needed because that the CHR6DM use

#include <Wire.h>
#include <Device_I2C.h>
#include <Axis.h>
#include <AQMath.h>
#include <Accelerometer_BMA180.h>

unsigned long timer;
Accelerometer_BMA180 accel;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Accelerometer library test (BMA180)");

  Wire.begin();
  
  accel.initialize();  
  accel.calibrate();
}

void loop() {
  
  if((millis() - timer) > 10) // 100Hz
  {
    timer = millis();
    accel.measure();
    
    Serial.print("Roll: ");
    Serial.print(accel.getMeterPerSec(XAXIS));
    Serial.print(" Pitch: ");
    Serial.print(accel.getMeterPerSec(YAXIS));
    Serial.print(" Yaw: ");
    Serial.print(accel.getMeterPerSec(ZAXIS));
    Serial.println();
  }

}
