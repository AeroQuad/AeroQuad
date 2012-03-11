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

#ifndef _AQ_BATTERY_MONITOR_
#define _AQ_BATTERY_MONITOR_

#include <BatteryMonitorTypes.h>

#define BM_WARNING_RATIO 1.1

byte    numberOfBatteries = 0; 
boolean batteryAlarm      = false;
boolean batteryWarning    = false;

unsigned short batteryAlarmCellVoltage   = 333; // 9.9V on 3S
unsigned short batteryWarningCellVoltage = 366; // 11.0V on 3S

void setBatteryCellVoltageThreshold(float alarmVoltage) {
  
  batteryAlarmCellVoltage   = alarmVoltage*100.0;
  batteryWarningCellVoltage = alarmVoltage*BM_WARNING_RATIO*100.0;
}

// Reset Battery statistics
void resetBattery(byte batno) {

  if (batno < numberOfBatteries) {
    batteryData[batno].voltage      = 1200;
#ifdef BM_EXTENDED
    batteryData[batno].minVoltage   = 9900;
    batteryData[batno].current      = 0;
    batteryData[batno].maxCurrent   = 0;
    batteryData[batno].usedCapacity = 0;
#endif
  }
}

void initializeBatteryMonitor(byte nb, float alarmVoltage) {

  numberOfBatteries = nb;
  setBatteryCellVoltageThreshold(alarmVoltage);
  for (int i = 0; i < numberOfBatteries; i++) {
    resetBattery(i);
  }
  measureBatteryVoltage(0); // Initial measurement
}

byte batteryGetCellCount(byte batNo) {
  if (batteryData[batNo].cells) {
    return batteryData[batNo].cells;
  }
  else if (batteryData[batNo].voltage < 500) {
    return 1;
  }
  else if (batteryData[batNo].voltage < 860) {
    return 2;
  }
  else {
    return 3;
  }
}

boolean batteryIsAlarm(byte batNo) {

  if (batteryData[batNo].voltage < batteryGetCellCount(batNo) * batteryAlarmCellVoltage) {
    return true;
  }
  return false;
}

boolean batteryIsWarning(byte batNo) {

  if (batteryData[batNo].voltage < batteryGetCellCount(batNo) * batteryWarningCellVoltage) {
    return true;
  }
  return false;
}

void measureBatteryVoltage(unsigned short deltaTime) {

  batteryAlarm = false;  
  batteryWarning = false;
  for (int i = 0; i < numberOfBatteries; i++) {
    batteryData[i].voltage = (long)analogRead(batteryData[i].vPin) * batteryData[i].vScale / 1024 + batteryData[i].vBias;
#ifdef BM_EXTENDED
    if (batteryData[i].voltage < batteryData[i].minVoltage) {
      batteryData[i].minVoltage = batteryData[i].voltage;
    }
    if (batteryData[i].cPin != BM_NOPIN) {
      batteryData[i].current = (long)analogRead(batteryData[i].cPin) * batteryData[i].cScale * 10 / 1024 + batteryData[i].cBias * 10;
      if (batteryData[i].current > batteryData[i].maxCurrent) { 
        batteryData[i].maxCurrent = batteryData[i].current;
      }
      // current in 10mA , time in ms -> usedCapacity in uAh  // i.e. / 360 <=> * ( 91 / 32768 )
      batteryData[i].usedCapacity += (long)batteryData[i].current * (long)deltaTime * 91 / 32768;
    }
#endif
    if (batteryIsAlarm(i)) {
      batteryAlarm = true;
    }
    if (batteryIsWarning(i)) {
      batteryWarning = true;
    }
  }  
}
#endif
