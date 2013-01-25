/*
  AeroQuad v3.1 - January 2013
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

//Choose how many channels you have available (6, 8 or 10)

//#define LASTCHANNEL 6
#define LASTCHANNEL 8
//#define LASTCHANNEL 10

// Uncomment only one of the following receiver types depending on the type you are using
// see http://aeroquad.com/showwiki.php?title=Connecting+the+receiver+to+your+AeroQuad+board
  
//PPM receivers
//#include <Receiver_PPM.h>   // for Arduino boards using a PPM receiver
//#include <Receiver_HWPPM.h> // for AeroQuad shield v1.x with Arduino Mega and shield v2.x with hardware mod                       
                              
//PWM receivers
//#include <Receiver_MEGA.h>   // for AeroQuad shield v1.x with Arduino Mega and shield v2.x using a standard PWM receiver
//#include <Receiver_328p.h> // for AeroQuad shield v1.x with Arduino Due/Uno and mini shield v1.0 using a standard PWM receiver

//Futaba sBus
#define sBus // for sBus receiver

// -------------  End of configuration ----------------- //


#if defined(sBus)
  #include <Receiver_SBUS.h>
#endif

unsigned long timer;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Receiver library test");

  initializeReceiver(LASTCHANNEL);   
  receiverXmitFactor = 1.0;
}

void loop() {
  
  if((millis() - timer) > 50) // 20Hz
  {
    timer = millis();
   
    #if defined(sBus)
      readSBUS();
    #endif
    
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
    Serial.print(" AUX1: ");
    Serial.print(receiverCommand[AUX1]);
    
    if(LASTCHANNEL == 8 || LASTCHANNEL == 10)  {
      
      Serial.print(" AUX2: ");
      Serial.print(receiverCommand[AUX2]);
      Serial.print(" AUX3: ");
      Serial.print(receiverCommand[AUX3]);

      if (LASTCHANNEL == 10)  {

        Serial.print(" AUX4: ");
        Serial.print(receiverCommand[AUX4]);
        Serial.print(" AUX5: ");
        Serial.print(receiverCommand[AUX5]);
      }
    }
    
    Serial.println();
  }
}
