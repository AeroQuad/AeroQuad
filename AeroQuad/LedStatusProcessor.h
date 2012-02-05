/*
  AeroQuad v3.0 - Febuary 2012
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)


#ifndef _AQ_LedProcessor_H_
#define _AQ_LedProcessor_H_

#define TWO_Hz_TIME 500
unsigned long lastLedChangeTime = 0;
byte flashingLedState = LOW;

void processLedStatus() {
 
  if ( (currentTime - lastLedChangeTime) > TWO_Hz_TIME) {
    flashingLedState = flashingLedState == LOW ? HIGH : LOW;
    lastLedChangeTime = currentTime;
  }
  
  //
  // process ready state light
  //
  #if defined (UseGPS)
    if (haveAGpsLock()) {
      digitalWrite(LED_Green, HIGH);
    }
    else { 
      digitalWrite(LED_Green, flashingLedState);
    }
  #endif
  
  #if defined (BattMonitor)
    if (batteryAlarm) {
      digitalWrite(LED_Red, flashingLedState);
    }
    else { 
      digitalWrite(LED_Red, LOW);
    }
  #endif  
}

#endif
