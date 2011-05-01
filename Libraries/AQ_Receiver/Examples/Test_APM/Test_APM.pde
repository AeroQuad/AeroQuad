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

#include <AQMath.h>
#include <Axis.h>
#include <Receiver_APM.h>


unsigned long timer;
Receiver_APM receiver;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Receiver library test (Receiver_APM)");

  receiver.initialize();  
}

void loop() {
  
  if((millis() - timer) > 50) // 20Hz
  {
    timer = millis();
    receiver.read();
    
    Serial.print("Throttle: ");
    Serial.print(receiver.getRaw(THROTTLE));
    Serial.print(" Yaw: ");
    Serial.print(receiver.getRaw(YAW));
    Serial.print(" Roll: ");
    Serial.print(receiver.getRaw(ROLL));
    Serial.print(" Pitch: ");
    Serial.print(receiver.getRaw(PITCH));
    Serial.print(" Mode: ");
    Serial.print(receiver.getRaw(MODE));
    Serial.print(" Aux: ");
    Serial.print(receiver.getRaw(AUX));
    Serial.println();
  }
}
