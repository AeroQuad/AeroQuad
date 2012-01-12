/*
  AeroQuad v3.0 - Nov 2011
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

#ifndef _AQ_OSD_MAX7456_BATTMONITOR_H_
#define _AQ_OSD_MAX7456_BATTMONITOR_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Battery voltage Display //////////////////////////
//////////////////////////////////////////////////////////////////////////////

#include "BatteryMonitorTypes.h"

byte    osdBatCounter = 0;
boolean descentWarningShown = false;

void displayVoltage(byte areMotorsArmed) {

  byte    osdBatNo     = osdBatCounter % numberOfBatteries;
  boolean osdBatMinMax = osdBatCounter / numberOfBatteries / 4;

  // don't show min/max when armed or we have not flown yet
  if ((areMotorsArmed == true) || (armedTime == 0)){
    osdBatMinMax = false;
  }

  int currentValue;
  if (osdBatMinMax) {
    currentValue = batteryData[osdBatNo].minVoltage*10.0;
  }
  else {
    currentValue = batteryData[osdBatNo].voltage*10.0;
  }

  char buf[12];
  snprintf(buf,7,"%c%2d.%1dV",(osdBatMinMax) ? '\23' : '\20', currentValue/10,currentValue%10);

  // Following blink only symbol on warning and all on alarm
  writeChars( buf,   1, batteryIsWarning(osdBatNo)?1:0, VOLTAGE_ROW + osdBatNo, VOLTAGE_COL );
  writeChars( buf+1, 5, batteryIsAlarm(osdBatNo)?1:0,   VOLTAGE_ROW + osdBatNo, VOLTAGE_COL + 1 );

  if (batteryData[osdBatNo].cPin != BM_NOPIN) {
    // current sensor installed
    if (osdBatMinMax) {
      currentValue = batteryData[osdBatNo].maxCurrent*10.0;
    }
    else {
      currentValue = batteryData[osdBatNo].current*10.0;
    }

    if (abs(currentValue)>=100) { // > 10A only display whole amps
      snprintf(buf,12,"%4dA%5d\24  ", currentValue/10, (int)batteryData[osdBatNo].usedCapacity);
    }
    else {
      snprintf(buf,12,"%c%1d.%1dA%5d\24  ", currentValue<0?'-':' ',abs(currentValue/10),abs(currentValue%10),(int)batteryData[osdBatNo].usedCapacity);
    }

    writeChars( buf, 11, 0, VOLTAGE_ROW+osdBatNo, VOLTAGE_COL+6 );
  }

  osdBatCounter++;
  if (osdBatCounter >= numberOfBatteries * 8) {
    osdBatCounter = 0;
  }

  #if defined (BattMonitorAutoDescent)
    if (batteryAlarm && areMotorsArmed) {
      if (!descentWarningShown) {
        notifyOSD(OSD_CENTER|OSD_CRIT|OSD_BLINK, "BAT. CRITICAL - DESCENTING");
        descentWarningShown = true;
      }
    }
    else {
      descentWarningShown = false;
    }
  #endif
}

#endif  // #define _AQ_OSD_MAX7456_BATTMONITOR_H_


