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

#ifndef _AQ_BATTERY_MONITOR_TYPES
#define _AQ_BATTERY_MONITOR_TYPES

#define BM_NOPIN 255

struct BatteryData {
  byte  vPin,cPin;        // A/D pins for voltage and current sensors (255 = BM_NOPIN <=> no sensor)
  byte  cells;            // Number of Cells (used for alarm/warning voltage
  float vScale,vBias;     // voltage polynom V = vbias + AnalogIn(vpin)*vscale
  float cScale,cBias;     // current polynom C = cbias + AnalogIn(cpin)*cscale
  float voltage;          // Current battery voltage
  float current;          // Current battery current
  float minVoltage;       // Minimum voltage since reset
  float maxCurrent;       // Maximum current since reset
  float usedCapacity;     // Capacity used since reset (in mAh)
};

extern struct BatteryData batteryData[];       // BatteryMonitor config, !! MUST BE DEFINED BY MAIN SKETCH !!
extern const byte         batteryBuzzerPins[]; // Pins for battery alarm buzzer/LED:s (terminated by 255)
extern byte               numberOfBatteries; // number of batteries monitored, defined by BatteryMonitor
extern boolean            batteryAlarm;      // any battery in alarm state used for e.g. autodescent
extern boolean            batteryWarning;    // any battery in warning state

// Helper macros to make battery definitions cleaner

// for defining battery with voltage and optional current sensors
#define DEFINE_BATTERY(CELLS,VPIN,VSCALE,VBIAS,CPIN,CSCALE,CBIAS) {VPIN,CPIN,CELLS,VSCALE,VBIAS, CSCALE, CBIAS, 0.0, 0.0, 0.0, 0.0, 0.0}

// Function declarations

boolean batteryIsAlarm(byte batteryNo);
boolean batteryIsWarning(byte batteryNo);
void resetBattery(byte batteryNo);
void initializeBatteryMonitor(byte numberOfMonitoredBatteries, float alarmVoltage);
void setBatteryCellVoltageThreshold(float alarmVoltage);
void measureBatteryVoltage(float deltaTime);
#endif
