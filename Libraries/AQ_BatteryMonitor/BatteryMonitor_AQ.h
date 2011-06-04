/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AQ_BATTERY_MONITOR_AQ_
#define _AQ_BATTERY_MONITOR_AQ_

#include "BatteryMonitor.h"
#include <WProgram.h>

#if defined (__AVR_ATmega328P__)
  #define BUZZERPIN 12
#else
  #define BUZZERPIN 49
#endif

class BatteryMonitor_AQ : public BatteryMonitor {
private:
  byte state, firstAlarm;
  float diode; // raw voltage goes through diode on Arduino
  float batteryScaleFactor;
  long currentBatteryTime, previousBatteryTime;

public:
  BatteryMonitor_AQ() : BatteryMonitor() {}

  void initialize(float diodeValue = 0.0) {
    float R1   = 15000;
    float R2   =  7500;
    float Aref =     5.0;
    batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));
	diode = diodeValue;
    analogReference(DEFAULT);
    pinMode(BUZZERPIN, OUTPUT); // connect a 12V buzzer to buzzer pin
    digitalWrite(BUZZERPIN, LOW);
    previousBatteryTime = millis();
    state = LOW;
    firstAlarm = OFF;
  }

  void lowBatteryEvent(byte level) {
    long currentBatteryTime = millis() - previousBatteryTime;
    if (level == OK) {
      digitalWrite(BUZZERPIN, LOW);
      autoDescent = 0;
    }
    if (level == WARNING) {
      if (currentBatteryTime > 1100) {
        //autoDescent = 50;
        digitalWrite(LED3PIN, HIGH);
        digitalWrite(BUZZERPIN, HIGH);
      }
      if (currentBatteryTime > 1200) {
        previousBatteryTime = millis();
        //autoDescent = 0;
        digitalWrite(LED3PIN, LOW);
        digitalWrite(BUZZERPIN, LOW);
      }
    }
    if (level == ALARM) {
      if (firstAlarm == OFF) autoDescent = 0; // intialize autoDescent to zero if first time in ALARM state
      firstAlarm = ON;
      digitalWrite(BUZZERPIN, HIGH); // enable buzzer
      digitalWrite(LED3PIN, HIGH);
      if ((currentBatteryTime > 500) && (throttle > 1400)) {
        autoDescent -= 1; // auto descend quad
        holdAltitude -= 0.2; // descend if in attitude hold mode
        previousBatteryTime = millis();
      }
    }
  }

  const float readBatteryVoltage(byte channel) {
    return (analogRead(channel) * batteryScaleFactor) + diode;
  }
};



#endif