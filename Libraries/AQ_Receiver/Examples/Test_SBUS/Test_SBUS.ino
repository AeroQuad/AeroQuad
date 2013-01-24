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
#include <GlobalDefined.h>
#include <Receiver_SBUS.h>

unsigned long timer;

void setup() {
  Serial.begin(115200);
  Serial.println("Receiver library test (Receiver_SBUS)");
  
  initializeReceiver();
}

void loop() {
  
  if((millis() - timer) > 20) // 50Hz
  {
    timer = millis();
    readSBUS();
    readReceiver();
    
    Serial.print("Throttle: ");
    Serial.print(receiverCommand[THROTTLE]);
    Serial.print(" Yaw: ");
    Serial.print(receiverCommand[ZAXIS]);
    Serial.print(" Roll: ");
    Serial.print(receiverCommand[XAXIS]);
    Serial.print(" Pitch: ");
    Serial.print(receiverCommand[YAXIS]);
    Serial.print(" Mode: ");
    Serial.print(receiverCommand[MODE]);
    Serial.print(" Aux1: ");
    Serial.print(receiverCommand[AUX1]);
    Serial.print(" Aux2: ");
    Serial.print(receiverCommand[AUX2]);
    Serial.print(" Aux3: ");
    Serial.println(receiverCommand[AUX3]);
  }
}

