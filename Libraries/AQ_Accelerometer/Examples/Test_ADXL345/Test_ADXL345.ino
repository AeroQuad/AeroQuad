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

#include <Wire.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>
#include <AQMath.h>

// if using 6DOF, uncomment this one:
#include <Accelerometer_ADXL345.h>  
// if using 9DOF, uncomment this one:
//#include <Accelerometer_ADXL345_9DOF.h 

unsigned long timer;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Accelerometer library test (ADXL345)");

  Wire.begin();
  
  initializeAccel();
  computeAccelBias();
  // changing the scale factor is necessary to get non-0 values from this test sketch
  // don't worry about how accurate the data is at this point, just make sure it changes as you move the board
  accelScaleFactor[XAXIS]=accelScaleFactor[YAXIS]=accelScaleFactor[ZAXIS]=-0.038;
}

void loop() {
  
  if((millis() - timer) > 10) // 100Hz
  {
    timer = millis();
    measureAccel();
    
    Serial.print("Roll: ");
    Serial.print(meterPerSecSec[XAXIS]);
    Serial.print(" Pitch: ");
    Serial.print(meterPerSecSec[YAXIS]);
    Serial.print(" Yaw: ");
    Serial.print(meterPerSecSec[ZAXIS]);
    Serial.println();
  }
}
