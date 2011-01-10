/*
  AeroQuad v2.1.3 Beta - December 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

// *************************************************************************
// ************************** Battery Monitor ******************************
// *************************************************************************
class BatteryMonitor {
public: 
  enum BatteryStatus {OK, Warning, Critical};
  
protected: 
  byte batteryPin;
  
  float lowVoltageWarning;  // Pack voltage at which to trigger alarm (first alarm)
  float lowVoltageCritical;    // Pack voltage at which to trigger alarm (critical alarm)
  BatteryStatus batteryStatus;
  
  float batteryVoltage;
  float batteryScaleFactor;
  
  float diode;
  
  //callback function
  void (*batteryStatusCallback)(BatteryStatus);

public:
  BatteryMonitor(void) { 
    lowVoltageWarning = 10.0; //10.8;
    lowVoltageCritical = 9.5; //10.2;
    diode = 0;
    batteryStatus = OK;
  }

  virtual void initialize(byte pin, float scaleFactor) {
     batteryPin = pin;
     batteryScaleFactor = scaleFactor;
  }
  
  // defined as virtual in case future hardware has custom way to read battery voltage
  virtual const float readBatteryVoltage(void) {
    return (analogRead(batteryPin) * batteryScaleFactor) + diode; 
  }
  
  //For people with diodes
  void setDiode(float value)
  {
    diode = value;
  }
  
  void setStatusCallback(void (*callback)(BatteryStatus)) {
    batteryStatusCallback = callback;
  }
  
  void measure(void) {
    #ifdef BattMonitor_SmoothVoltage
      batteryVoltage = smooth(readBatteryVoltage(), batteryVoltage, 0.1);
    #else
      batteryVoltage = readBatteryVoltage();
    #endif

    if (batteryVoltage < lowVoltageCritical) batteryStatus = Critical;
    else if (batteryVoltage < lowVoltageWarning) batteryStatus = Warning;
    else batteryStatus = OK;
    //Fire the external event
    if(batteryStatusCallback != NULL)
      batteryStatusCallback(batteryStatus);
  }

  const float getData(void) {
    return batteryVoltage;
  }
  
  const BatteryStatus getStatus(void) {
    return batteryStatus;
  }
};

#if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)

// *******************************************************************************
// ********************** AeroQuad_Wii Battery Monitor ***************************
// *******************************************************************************
class BatteryMonitor_Wii : public BatteryMonitor {

public: 
  BatteryMonitor_Wii() : BatteryMonitor(){}

  //Read the voltage from the Nunchuck's Stick X value
  const float readBatteryVoltage(byte channel) {
    return NWMP_sx * batteryScaleFactor; 
  }
};

#endif

//Event fired when battery voltage is measured
void batteryStatusEvent(BatteryMonitor::BatteryStatus batteryStatus) {
    //SERIAL_PORT->println(batteryStatus);
    switch(batteryStatus)
    {
     case BatteryMonitor::OK:
       leds.alwaysOn();
      break; 
     case BatteryMonitor::Warning:
       leds.flashSlow();
      break; 
     case BatteryMonitor::Critical:
       leds.flashFast();
      break; 
    }
}


