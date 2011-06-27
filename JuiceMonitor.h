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

  struct batteryconfig {
    byte vpin,cpin;         // A/D pins for voltage and current sensors (255 == no sensor)
    float vwarning,valarm;  // Warning and Alarm voltage level
    float vscale,vbias;     // voltage polynom V = vbias + (Aref*Ain(vpin))*vscale;
    float cscale,cbias;     // current polynom C = cbias + (Aref*Ain(cpin))*cscale;
  };
#define NC 255              // NotConnected - use this in place of missing (current) sensor

// USER CONFIGURATION BELOW
#define AREF 5.0

  // below defines two batteries, ALWAYS modify this for your own configuration !!! 
  // battery 1 
  //   (vpin)     = 0   - voltage divider on v2.0 shield
  //   (cpin)     = NC  - no current sensor
  //   (vwarning) = 7.4 - warning voltage (for 2S LiPo)
  //   (valarm)   = 7.2 - alarm voltage
  //   (vscale)   = ((AREF / 1024.0) * (15.0+7.5)/7.5) - voltage divider on AQ v2.0 shield ( in - 15kOhm - out - 7.5kOhm - GND )
  //   (vbias)    = 0.82 - voltage drop on the diode on ArduinoMega VIN
  //   (cscale)   = 0.0 - N/A due to no sensor
  //   (cbias)    = 0.0 - N/A due to no sensor
  // battery 2
  //   (vpin)     = 1   - AIN1 on v2.0 shield
  //   (cpin)     = 2   - AIN2 on v2.0 shield
  //   (vwarning) = 11.1 - warning voltage (for 3S LiPo)
  //   (valarm)   = 10.5 - alarm voltage
  //   (vscale)   = ((AREF / 1024.0) * (15.0+10.0)/10.0) - voltage divider ( in - 15kOhm - out - 10kOhm - GND )
  //   (vbias)    = 0.0 - no diode
  //   (cscale)   = 100.0/1024.0 - a flytron 100A sensor giving 5V (=AREF) at 100A
  //   (cbias)    = 0.0 - no bias on this sensor
  
 const struct batteryconfig batconfig[] = {
   { 0, NC,  7.4, 7.2,   ((AREF / 1024.0) * (15.0 + 7.5) / 7.5), 0.82,          0.0, 0.0 },
   { 1,  2, 11.1, 10.5, ((AREF / 1024.0) * (15.0 + 10.0) / 10.0),  0.0, 100.0/1024.0, 0.0 }
};

// USER CONFIGURATION END

#define BATTERIES ( sizeof(batconfig) / sizeof(struct batteryconfig) )       // number of batteries to monitor, normally 1-3

class JuiceMonitor {
public:
  #define OK 0
  #define WARNING 1
  #define ALARM 2
  byte batteryStatus[BATTERIES];
  byte juiceStatus; // all batteries combined
  float batteryVoltage[BATTERIES];
  float batteryCurrent[BATTERIES];
  float batterymAh[BATTERIES];

  JuiceMonitor(void) {
    for (byte i; i<BATTERIES; i++) {
      batteryVoltage[i] = batconfig[i].vwarning + 1.0;
      batteryStatus[i] = OK;
      batterymAh[i] = 0.0;
    }
  }

  virtual void initialize(void);
  virtual const float readBatteryVoltage(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  virtual const float readBatteryCurrent(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  virtual void lowBatteryEvent(byte);

  void measure(byte armed) {
    juiceStatus=OK;
    for (byte i=0; i<BATTERIES;i++) {
      batteryVoltage[i] = filterSmooth(readBatteryVoltage(i), batteryVoltage[i], 0.1);
      if (armed == ON) {
        if (batteryVoltage[i] < batconfig[i].vwarning) batteryStatus[i] = WARNING;
        if (batteryVoltage[i] < batconfig[i].valarm) batteryStatus[i] = ALARM;
      } else {
        batteryStatus[i] = OK;
      }
      juiceStatus|=batteryStatus[i];
      
      batteryCurrent[i] = readBatteryCurrent(i);

      batterymAh[i] += G_Dt * batteryCurrent[i] / 3.6;
      // G_Dt is seconds , batteryCurrent in amps need to convert to mAh it divide by 3.6

    }
    if (3==juiceStatus) juiceStatus=ALARM; // fixup if both active
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
    return batteryStatus[channel];
  }
  const byte isI(byte channel) { // return 1 if current sensor is present (scale != 0)
    return (NC!=batconfig[channel].cpin);
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
  long currentBatteryTime, previousBatteryTime;

public:
  JuiceMonitor_AeroQuad() : JuiceMonitor(){}

  void initialize(void) {
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
    return ((float)analogRead(batconfig[channel].vpin) * batconfig[channel].vscale) + batconfig[channel].vbias;
  }
  const float readBatteryCurrent(byte channel) {
    if (isI(channel)) {
      return ((float)analogRead(batconfig[channel].cpin) * batconfig[channel].cscale) + batconfig[channel].cbias;
    } else {
      return 0.0;
    }
  }
};
