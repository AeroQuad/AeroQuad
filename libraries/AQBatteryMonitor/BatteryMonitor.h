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

#ifndef _AQ_BETTERY_MONITOR_H_
#define _AQ_BETTERY_MONITOR_H_

#define BATTERYPIN 0      // Ain 0 (universal to every Arduino), pin 55 on Mega (1280)
#define OK 0
#define WARNING 1
#define ALARM 2


// *************************************************************************
// ************************** Battery Monitor ******************************
// *************************************************************************
class BatteryMonitor 
{
private:
  byte _batteryStatus;
  float _lowVoltageWarning;  // Pack voltage at which to trigger alarm (first alarm)
  float _lowVoltageAlarm;    // Pack voltage at which to trigger alarm (critical alarm)
  float _batteryVoltage;

public:  

  BatteryMonitor() 
  {
    _lowVoltageWarning = 10.2; //10.8;
    _lowVoltageAlarm = 9.5; //10.2;
    _batteryVoltage = _lowVoltageWarning + 2;
    _batteryStatus = OK;
  }

  virtual void initialize();
  virtual const float readBatteryVoltage(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  virtual void lowBatteryEvent(byte);

  void measure(byte armed) 
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
    lowBatteryEvent(_batteryStatus);
  }

  const float getData() 
  {
    return _batteryVoltage;
  }
};

#endif

