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


// use this if using SparkFun 9DOF, comment out otherwise
#define SPARKFUN_9DOF_5883L

#include <AQMath.h>
#include <GlobalDefined.h>
#include <SensorsStatus.h>
#include <Magnetometer_HMC5883L.h>

unsigned long timer100Hz = 0;
unsigned long timer25Hz = 0;

void setup() {
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("Magnetometer library test (HMC5883L)");
  
  Wire.begin();
  initializeMagnetometer();
  if (vehicleState & MAG_DETECTED) {
    Serial.println("Magnetometer found");
    delay(1000);
  } else {
    Serial.println("!! Magnetometer not found !!");
  }
}

void loop() {
  
  if (vehicleState & MAG_DETECTED)
  {
    if ((millis() - timer100Hz) > 10) // 100Hz
    {
      timer100Hz = millis();
      measureMagnetometer(0.0,0.0);
    }
     
    if ((millis() - timer25Hz) > 40) // 25Hz 
    {
      timer25Hz = millis();
      
      Serial.print("Roll: ");
      Serial.print(getMagnetometerRawData(YAXIS));
      Serial.print(", Pitch: ");
      Serial.print(getMagnetometerRawData(XAXIS));
      Serial.print(", Yaw: ");
      Serial.print(getMagnetometerRawData(ZAXIS));
      Serial.println();
    }
  }
}
