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

#include "WProgram.h"

#define BATTERYPIN 0      // Ain 0 (universal to every Arduino), pin 55 on Mega (1280)
#define OK 0
#define WARNING 1
#define ALARM 2

#define ON 1
#define OFF 0


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
 
protected:
  int _autoDescent;
  virtual void lowBatteryEvent(byte,int);
 
public:  

  BatteryMonitor();

  virtual void initialize();
  virtual const float readBatteryVoltage(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  void measure(byte armed,int throttle);
  const float getData();
  const int getAutoDescent();
  
};

#endif

