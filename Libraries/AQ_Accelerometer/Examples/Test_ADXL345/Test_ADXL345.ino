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

/* 
  If using a SparkFun 9DOF, use Accelerometer_ADXL345_9DOF.h
  otherwise, use Accelerometer_ADXL345.h
*/
//#include <Accelerometer_ADXL345.h>
#include <Accelerometer_ADXL345_9DOF.h>

unsigned long timer100Hz = 0;
unsigned long timer25Hz = 0;

void setup() 
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Accelerometer library test (ADXL345)");

  Wire.begin();
  
  initializeAccel();
  if (vehicleState & ACCEL_DETECTED) {
    Serial.println("Accelerometer found");
  } else {
    Serial.println("!! Accelerometer not found !!");
  }
  computeAccelBias();
}

void loop() 
{   
  if (vehicleState & ACCEL_DETECTED)
  {
    measureAccelSum();
    
    if ((millis() - timer100Hz) > 10) // 100Hz
    {
      timer100Hz = millis();
      evaluateMetersPerSec();
    }
      
    if ((millis() - timer25Hz) > 40) // 25Hz
    {
      timer25Hz = millis();
      
      Serial.print("Roll: ");
      Serial.print(meterPerSecSec[XAXIS]);
      Serial.print(" Pitch: ");
      Serial.print(meterPerSecSec[YAXIS]);
      Serial.print(" Yaw: ");
      Serial.println(meterPerSecSec[ZAXIS]);
    }
  }
}
