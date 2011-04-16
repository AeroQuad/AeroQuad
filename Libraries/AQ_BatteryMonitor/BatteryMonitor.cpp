/*
  AeroQuad v2.2 - Feburary 2011
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

#include "BatteryMonitor.h"

#include <AQMath.h>

BatteryMonitor::BatteryMonitor() 
{
  _lowVoltageWarning = 10.2; //10.8;
  _lowVoltageAlarm = 9.5; //10.2;
  _batteryVoltage = _lowVoltageWarning + 2;
  _batteryStatus = OK;
  _autoDescent = 0;
}

void BatteryMonitor::initialize() {}
const float BatteryMonitor::readBatteryVoltage(byte) {} // defined as virtual in case future hardware has custom way to read battery voltage
void BatteryMonitor::lowBatteryEvent(byte,int) {}

void BatteryMonitor::measure(byte armed,int throttle) 
{
  _batteryVoltage = filterSmooth(readBatteryVoltage(BATTERYPIN), _batteryVoltage, 0.1);
  if (armed == ON) 
  {
    if (_batteryVoltage < _lowVoltageWarning) 
    {
      _batteryStatus = WARNING;
    }
    if (_batteryVoltage < _lowVoltageAlarm)
    {
      _batteryStatus = ALARM;
    }
  }
  else
  {
    _batteryStatus = OK;
  }
  lowBatteryEvent(_batteryStatus,throttle);
}

const float BatteryMonitor::getData() 
{
  return _batteryVoltage;
}

const int BatteryMonitor::getAutoDescent()
{
  return _autoDescent;
}
