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


#include "APMBatteryMonitor.h"


#define FL_LED 57 // Ain 2 on Mega
#define FR_LED 58 // Ain 3 on Mega
#define RR_LED 59 // Ain 4 on Mega
#define RL_LED 60 // Ain 5 on Mega
#define LEDDELAY 200

APMBatteryMonitor::APMBatteryMonitor()
{
}

void APMBatteryMonitor::ledCW()
{
  digitalWrite(RL_LED, HIGH);
  delay(LEDDELAY);
  digitalWrite(RL_LED, LOW);
  digitalWrite(RR_LED, HIGH);
  delay(LEDDELAY);
  digitalWrite(RR_LED, LOW);
  digitalWrite(FR_LED, HIGH);
  delay(LEDDELAY);
  digitalWrite(FR_LED, LOW);
  digitalWrite(FL_LED, HIGH);
  delay(LEDDELAY);
  digitalWrite(FL_LED, LOW);
};

void APMBatteryMonitor::ledsON()
{
  digitalWrite(RL_LED, HIGH);
  digitalWrite(RR_LED, HIGH);
  digitalWrite(FR_LED, HIGH);
  digitalWrite(FL_LED, HIGH);
};

void APMBatteryMonitor::ledsOFF()
{
  digitalWrite(RL_LED, LOW);
  digitalWrite(RR_LED, LOW);
  digitalWrite(FR_LED, LOW);
  digitalWrite(FL_LED, LOW);
};

  
  
void APMBatteryMonitor::initialize() 
{
  float R1   = 10050; //the SMD 10k resistor measured with DMM
  float R2   =  3260; //3k3 user mounted resistor measured with DMM
  float Aref = 3.27F; //AREF 3V3 used (solder jumper) and measured with DMM
  _batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));

  _diode = 0.306F; //Schottky diode on APM board, drop measured with DMM

  analogReference(EXTERNAL); //use Oilpan 3V3 AREF or if wanted, define DEFAULT here to use VCC as reference and define that voltage in BatteryReadArmLed.h

  pinMode(FL_LED ,OUTPUT);
  pinMode(FR_LED ,OUTPUT);
  pinMode(RR_LED ,OUTPUT);
  pinMode(RL_LED ,OUTPUT);

  //batteryVoltage = readBatteryVoltage(BATTERYPIN);
}

void APMBatteryMonitor::lowBatteryEvent(byte level,int throttle) // <- this logic by Jose Julio
{  
  static byte batteryCounter=0;
  byte freq;

  if (level == OK) 
  {
    ledsON();
    _autoDescent = 0; //reset autoAscent if battery is good
  }
  else
  {
    batteryCounter++;
    if (level == WARNING) 
    {
      freq = 40;  //4 seconds wait
    }
    else 
    {
      freq = 5; //0.5 second wait
    }
    if (batteryCounter < 2) 
    {
      ledsOFF();  //indicate with led's everytime autoDescent kicks in
    }
    #if defined(AltitudeHold)
      if (throttle > 1400) 
      {
        _holdAltitude -= 0.2; //-0.2m in 2 fixed rates, one where battery < 10.8V and one where battery < 10.2V, only done if in altitude hold mode
      }
    #else
      if (throttle > 1400) 
      {
        _autoDescent -= 2; //will remove 2µs throttle every time led's blink in two speeds (10.8 and 10.2V) as long as there is throttle to lower
      }
    #endif
    else if (batteryCounter < freq) 
    {
      ledsON();
    }
    else
    {
      batteryCounter = 0;
    }
  }
}

const float APMBatteryMonitor::readBatteryVoltage(byte channel) 
{
  return (analogRead(channel) * _batteryScaleFactor) + _diode;
}
