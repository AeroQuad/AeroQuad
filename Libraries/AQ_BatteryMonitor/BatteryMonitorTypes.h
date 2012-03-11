/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#if ! defined (__AVR_ATmega328P__) && ! defined(__AVR_ATmegaUNO__)
  #define BM_EXTENDED
#endif

#define BM_NOPIN 255

struct BatteryData {
  byte  vPin;                   // A/D pin for voltage sensor
  byte  cells;                  // Number of Cells (used for alarm/warning voltage
  short vScale,vBias;  // voltage polynom V = vbias + AnalogIn(vpin)*vscale
  unsigned short voltage;       // Current battery voltage (in 10mV:s)
#ifdef BM_EXTENDED
  unsigned short minVoltage;    // Minimum voltage since reset
  byte  cPin;             // A/D pin for current sensor (255 = BM_NOPIN <=> no sensor)
  short cScale,cBias;     // current polynom C = cbias + AnalogIn(cpin)*cscale
  short current;          // Current battery current (in 10mA:s)
  short maxCurrent;       // Maximum current since reset
  long  usedCapacity;     // Capacity used since reset (in uAh)
#endif
};

extern struct BatteryData batteryData[];       // BatteryMonitor config, !! MUST BE DEFINED BY MAIN SKETCH !!
extern byte               numberOfBatteries; // number of batteries monitored, defined by BatteryMonitor
extern boolean            batteryAlarm;      // any battery in alarm state used for e.g. autodescent
extern boolean            batteryWarning;    // any battery in warning state

// Helper macros to make battery definitions cleaner

// for defining battery with voltage and optional current sensors
#ifdef BM_EXTENDED
#define DEFINE_BATTERY(CELLS,VPIN,VSCALE,VBIAS,CPIN,CSCALE,CBIAS) {(VPIN),(CELLS),(VSCALE*100.0),(VBIAS*100.0),0,0,(CPIN),(CSCALE*10.0),(CBIAS*10.0),0,0,0}
#else
#define DEFINE_BATTERY(CELLS,VPIN,VSCALE,VBIAS,CPIN,CSCALE,CBIAS) {(VPIN),(CELLS),(VSCALE*100.0),(VBIAS*100.0),0}
#endif
// Function declarations

boolean batteryIsAlarm(byte batteryNo);
boolean batteryIsWarning(byte batteryNo);
void resetBattery(byte batteryNo);
void initializeBatteryMonitor(byte numberOfMonitoredBatteries, float alarmVoltage);
void setBatteryCellVoltageThreshold(float alarmVoltage);
void measureBatteryVoltage(unsigned short deltaTime);
#endif
