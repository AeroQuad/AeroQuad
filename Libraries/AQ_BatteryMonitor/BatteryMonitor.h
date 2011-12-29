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

#ifndef _AQ_BATTERY_MONITOR_
#define _AQ_BATTERY_MONITOR_

#include <BatteryMonitorTypes.h>

#define BM_WARNING_RATIO 1.1

byte    numberOfBatteries = 0; 
boolean batteryAlarm      = false;

float batteryAlarmCellVoltage   = 3.33; // 10.0V on 3S
float batteryWarningCellVoltage = 3.66; // 11.0V on 3S

void setBatteryCellVoltageThreshold(float alarmVoltage) {
  
  batteryAlarmCellVoltage   = alarmVoltage;
  batteryWarningCellVoltage = alarmVoltage*BM_WARNING_RATIO;
}

// Reset Battery statistics
void resetBattery(byte batno) {

  if (batno < numberOfBatteries) {
    batteryData[batno].voltage      = 12.0;
    batteryData[batno].minVoltage   = 99.0;
    batteryData[batno].current      = 0.0;
    batteryData[batno].maxCurrent   = 0.0;
    batteryData[batno].usedCapacity = 0.0;
  }
}

void initializeBatteryMonitor(byte nb, float alarmVoltage) {

  numberOfBatteries = nb;
  setBatteryCellVoltageThreshold(alarmVoltage);
  for (int i = 0; i < numberOfBatteries; i++) {
    resetBattery(i);
  }
  measureBatteryVoltage(0.0); // Initial measurement
}

byte batteryGetCellCount(byte batNo) {
  if (batteryData[batNo].cells) {
    return batteryData[batNo].cells;
  }
  else if (batteryData[batNo].voltage<5.0) {
    return 1;
  }
  else if (batteryData[batNo].voltage<8.6) {
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


void measureBatteryVoltage(float deltaTime) {

  batteryAlarm = false;  
  for (int i = 0; i < numberOfBatteries; i++) {
    batteryData[i].voltage = (float)analogRead(batteryData[i].vPin) * batteryData[i].vScale + batteryData[i].vBias;
    if (batteryData[i].voltage < batteryData[i].minVoltage) {
      batteryData[i].minVoltage = batteryData[i].voltage;
    }
    if (batteryData[i].cPin != BM_NOPIN) {
      batteryData[i].current =  (float)analogRead(batteryData[i].cPin) * batteryData[i].cScale + batteryData[i].cBias;
      if (batteryData[i].current > batteryData[i].maxCurrent) { 
        batteryData[i].maxCurrent = batteryData[i].current;
      }
      batteryData[i].usedCapacity += batteryData[i].current * deltaTime / 3.6; // current(A) * 1000 * time(s) / 3600 -> mAh 
    }
    if (batteryIsAlarm(i)) {
      batteryAlarm = true;
    }
  }  
}
#endif