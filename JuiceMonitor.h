/*
  AeroQuad v2.x.x - June 2011
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

// Written by Kari: http://aeroquad.com/showthread.php?

// *************************************************************************
// ************************** Battery Monitor ******************************
// *************************************************************************

#ifdef BattMonitor
#error JuiceMonitor is not compatible with BatteryMonitor
#endif

class JuiceMonitor {
public:

  #define BATTERIES 2       // number of batteries to monitor, normally 1-3
  byte voltagePin[BATTERIES]; // AIN0,AIN1
  byte currentPin[BATTERIES]; // AIN2,AIN3
  #define OK 0
  #define WARNING 1
  #define ALARM 2
  byte batteryStatus[BATTERIES];
  byte juiceStatus; // all batteries combined
  float lowVoltageWarning[BATTERIES];  // Pack voltage at which to trigger alarm (first alarm)
  float lowVoltageAlarm[BATTERIES];    // Pack voltage at which to trigger alarm (critical alarm)
  float batteryVoltage[BATTERIES];
  float batteryCurrent[BATTERIES];
  float batterymAh[BATTERIES];

  JuiceMonitor(void) {
    voltagePin[0] = 0; // AIN0
    voltagePin[1] = 1; // AIN1
    currentPin[0] = 0; // N/A not connected
    currentPin[1] = 2; // AIN2
    lowVoltageWarning[0] = 7.0; //
    lowVoltageWarning[1] = 10.2; //
    lowVoltageAlarm[0] = 6.8; //
    lowVoltageAlarm[1] = 9.5; //
    for (byte i; i<BATTERIES; i++) {
      batteryVoltage[i] = lowVoltageWarning[i] + 1.0;
      batteryStatus[i] = OK;
      batterymAh[i] = 0.0;
    }
  }

  virtual void initialize(void);
  virtual const float readBatteryVoltage(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  virtual const float readBatteryCurrent(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  virtual void lowBatteryEvent(byte);
  virtual const byte isI(byte channel);

  void measure(byte armed) {
    juiceStatus=OK;
    for (byte i=0; i<BATTERIES;i++) {
      batteryVoltage[i] = filterSmooth(readBatteryVoltage(i), batteryVoltage[i], 0.1);
      if (armed == ON) {
        if (batteryVoltage[i] < lowVoltageWarning[i]) batteryStatus[i] = WARNING;
        if (batteryVoltage[i] < lowVoltageAlarm[i]) batteryStatus[i] = ALARM;
      } else {
        batteryStatus[i] = OK;
      }
      juiceStatus|=batteryStatus[i];
      
      batteryCurrent[i] = readBatteryCurrent(i);

      batterymAh[i] += G_Dt * batteryCurrent[i] / 3.6;
      // G_Dt is seconds , batteryCurrent in amps need to convert to mAh it divide by 3.6

    }
    if (juiceStatus=3) juiceStatus=ALARM; // fixup if both active
    lowBatteryEvent(juiceStatus);
  }  

  const float getU(byte channel) {
    return batteryVoltage[channel];
  }
  const float getI(byte channel) {
    return batteryCurrent[channel];
  }
  const float getC(byte channel) {
    return batterymAh[channel];
  }
  const byte getA(byte channel) {
    if (batteryVoltage[channel]<lowVoltageAlarm[channel])
      return ALARM;
    else if (batteryVoltage[channel]<lowVoltageWarning[channel])
      return WARNING;
    else
      return OK;
  }
  const float batteries(void) {
    return BATTERIES;
  }
};

// *******************************************************************************
// ************************ AeroQuad Juice Monitor *****************************
// *******************************************************************************
class JuiceMonitor_AeroQuad : public JuiceMonitor {
private:
  #define BUZZERPIN 49

  byte state, firstAlarm;
  float voltageDiode[BATTERIES]; // raw voltage goes through diode on Arduino
  float voltageScaleFactor[BATTERIES];
  float currentScaleFactor[BATTERIES];
  long currentBatteryTime, previousBatteryTime;

public:
  JuiceMonitor_AeroQuad() : JuiceMonitor(){}

  void initialize(void) {
    #define Aref 5.0
    voltageScaleFactor[0] = ((Aref / 1024.0) * ((15000.0 + 7500.0) / 7500.0));
    voltageDiode[0] = 0.82; // voltage drop on Arduino vin
    currentScaleFactor[0] = 0; // No sensor 

    voltageScaleFactor[1] = ((Aref / 1024.0) * ((15000.0 + 10000.0) / 10000.0));
    voltageDiode[1] = 0.0; // no diode here just resistors
    currentScaleFactor[1] = (100.0/1024.0); // Aref @ 100A

    analogReference(DEFAULT);
    pinMode(BUZZERPIN, OUTPUT); // connect a 12V buzzer to buzzer pin
    digitalWrite(BUZZERPIN, LOW);
    previousBatteryTime = millis();
    state = LOW;
    firstAlarm = OFF;
  }

  void lowBatteryEvent(byte level) {
    long currentBatteryTime = millis() - previousBatteryTime;
    if (level == OK) {
      digitalWrite(BUZZERPIN, LOW);
      autoDescent = 0;
    }
    if (level == WARNING) {
      //if ((autoDescent == 0) && (currentBatteryTime > 1000)) {
      //  autoDescent = -50;
      //}
      if (currentBatteryTime > 1100) {
        //autoDescent = 50;
        digitalWrite(LED3PIN, HIGH);
        digitalWrite(BUZZERPIN, HIGH);
      }
      if (currentBatteryTime > 1200) {
        previousBatteryTime = millis();
        //autoDescent = 0;
        digitalWrite(LED3PIN, LOW);
        digitalWrite(BUZZERPIN, LOW);
      }
    }
    if (level == ALARM) {
      if (firstAlarm == OFF) autoDescent = 0; // intialize autoDescent to zero if first time in ALARM state
      firstAlarm = ON;
      digitalWrite(BUZZERPIN, HIGH); // enable buzzer
      digitalWrite(LED3PIN, HIGH);
      if ((currentBatteryTime > 500) && (throttle > 1400)) {
        autoDescent -= 1; // auto descend quad
        holdAltitude -= 0.2; // descend if in attitude hold mode
        previousBatteryTime = millis();
        //if (state == LOW) state = HIGH;
        //else state = LOW;
        //digitalWrite(LEDPIN, state);
        //digitalWrite(LED2PIN, state);
        //digitalWrite(LED3PIN, state);
      }
    }
  }

  const float readBatteryVoltage(byte channel) {
    return ((float)analogRead(voltagePin[channel]) * voltageScaleFactor[channel]) + voltageDiode[channel];
  }
  const float readBatteryCurrent(byte channel) {
    if (isI(channel)) {
      return ((float)analogRead(currentPin[channel]) * currentScaleFactor[channel]);
    } else {
      return 0.0;
    }
  }
  const byte isI(byte channel) { // return 1 if current sensor is present (scale != 0)
    return currentScaleFactor[channel]!=0.0;
  }

};
