/*
  AeroQuad v2.1 - January 2011
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

// Written by Honk: http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13282#post13282

#ifndef _BATTERY_MONITOR_AEROQUAD_H_
#define _BATTERY_MONITOR_AEROQUAD_H_

#include <BatteryMonitor.h>

// *******************************************************************************
// ************************ AeroQuad Battery Monitor *****************************
// *******************************************************************************
class BatteryMonitorAeroQuad : public BatteryMonitor {
private:
  #if defined (__AVR_ATmega328P__)
    #define BUZZERPIN 12
  #else
    #define BUZZERPIN 49
  #endif
  long previousTime;
  byte state;
  float diode; // raw voltage goes through diode on Arduino
  float batteryScaleFactor;

public:
  BatteryMonitorAeroQuad() : BatteryMonitor(){}

  void initialize(void) {
    float R1   = 15000;
    float R2   =  7500;
    float Aref =     5.0;
    batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));    
    diode = 0.9; // measured with DMM
    analogReference(DEFAULT);
    pinMode(BUZZERPIN, OUTPUT); // connect a 12V buzzer to pin 49
    digitalWrite(BUZZERPIN, LOW);
    previousTime = millis();
    state = LOW;
  }

  void lowBatteryEvent(byte level) {
    long currentTime = millis()- previousTime;
    if (level == OK) {
      digitalWrite(BUZZERPIN, LOW);
      autoDescent = 0;
      //holdAltitude = 0;
    }
    if (level == WARNING) {
      if ((autoDescent == 0) && (currentTime > 1000)) {
        autoDescent = -50;
      }
      if (currentTime > 1100) {
        autoDescent = 50;
        digitalWrite(LED2PIN, HIGH);
        digitalWrite(BUZZERPIN, HIGH);
      }
      if (currentTime > 1200) {
        previousTime = millis();
        autoDescent = 0;
        digitalWrite(LED2PIN, LOW);
        digitalWrite(BUZZERPIN, LOW);
      }
    }
    if (level == ALARM) {
      if (digitalRead(BUZZERPIN) == LOW) autoDescent = 0; // intialize autoDescent to zero if first time in ALARM state
      digitalWrite(BUZZERPIN, HIGH); // enable buzzer
      if ((currentTime > 500) && (throttle > 1400)) {
        autoDescent -= 1; // auto descend quad
        holdAltitude -= 0.2; // descend if in attitude hold mode
        previousTime = millis();
        if (state == LOW) state = HIGH;
        else state = LOW;
        digitalWrite(LEDPIN, state);
        digitalWrite(LED2PIN, state);
        digitalWrite(LED3PIN, state);
      }
    }
  }

  const float readBatteryVoltage(byte channel) {
    return (analogRead(channel) * batteryScaleFactor) + diode;
  }
};

#endif