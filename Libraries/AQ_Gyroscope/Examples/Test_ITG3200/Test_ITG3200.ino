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

#include <AQMath.h>
#include <Device_I2C.h>
#include <GlobalDefined.h>

/* 
  If using a SparkFun 9DOF, use GyroScope_ITG3200_9DOF.h and ITG3200_ADDRESS_ALTERNATE
  If using a SparkFun 6DOF, use Gyroscope_ITG3200.h and ITG3200_ADDRESS_ALTERNATE
  otherwise, comment those both out and use Gyroscope_ITG3200.h
*/
#include <Gyroscope_ITG3200.h>
//#define ITG3200_ADDRESS_ALTERNATE
//#include <Gyroscope_ITG3200_9DOF.h>

unsigned long timer100Hz = 0;
unsigned long timer25Hz = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Gyroscope library test (ITG3200)");

  Wire.begin();

  initializeGyro();
  if (vehicleState & GYRO_DETECTED) {
    Serial.println("Gyroscope found");
  } else {
    Serial.println("!! Gyroscope not found !!");
  }
  calibrateGyro();
}

void loop() 
{
  if (vehicleState & GYRO_DETECTED)
  {
    measureGyroSum();
    
    if ((millis() - timer100Hz) > 10) // 100Hz
    {
      timer100Hz = millis();
      evaluateGyroRate();
    }
    
    if ((millis() - timer25Hz) > 40) // 25Hz
    {
      timer25Hz = millis();
      
      Serial.print("Roll: ");
      Serial.print(degrees(gyroRate[XAXIS]));
      Serial.print(" Pitch: ");
      Serial.print(degrees(gyroRate[YAXIS]));
      Serial.print(" Yaw: ");
      Serial.print(degrees(gyroRate[ZAXIS]));
      Serial.print(" Heading: ");
      Serial.println(degrees(gyroHeading));
    } 
  }
}
