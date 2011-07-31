/*
  AeroQuad v2.5 Beta 1 - July 2011
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

// *************************************************************************
// ************************** Battery Monitor ******************************
// *************************************************************************
class BatteryMonitor {
public:
  #define BATTERYPIN 0      // Ain 0 (universal to every Arduino), pin 55 on Mega (1280)
  #define OK 0
  #define WARNING 1
  #define ALARM 2
  byte batteryStatus;
  float lowVoltageWarning;  // Pack voltage at which to trigger alarm (first alarm)
  float lowVoltageAlarm;    // Pack voltage at which to trigger alarm (critical alarm)
  float batteryVoltage;

  BatteryMonitor(void) {
    lowVoltageWarning = 10.2; //10.8;
    lowVoltageAlarm = 9.5; //10.2;
    batteryVoltage = lowVoltageWarning + 2;
    batteryStatus = OK;
  }

  virtual void initialize(void);
  virtual const float readBatteryVoltage(byte); // defined as virtual in case future hardware has custom way to read battery voltage
  virtual void lowBatteryEvent(byte);

  void measure(byte armed) {
    batteryVoltage = filterSmooth(readBatteryVoltage(BATTERYPIN), batteryVoltage, 0.1);
    if (armed == ON) {
      if (batteryVoltage < lowVoltageWarning) batteryStatus = WARNING;
      if (batteryVoltage < lowVoltageAlarm) batteryStatus = ALARM;
    }
    else
      batteryStatus = OK;
    lowBatteryEvent(batteryStatus);
  }

  const float getData(void) {
    return batteryVoltage;
  }
};

// Definition below requested by Ala42 to make code compilable for non-AVR platforms
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

// ***********************************************************************************
// ************************ BatteryMonitor APM & CHR6DM  *****************************
// ***********************************************************************************
/* Circuit:

  Vin--D1--R1--|--R2--GND
               |
               |
              Vout
*/
//If lipo is 12.6V and diode drop is 0.6V (res 12.0V), the voltage from divider network will be = 2.977V
//calculation: AREF/1024.0 is Vout of divider network
//Vin = lipo voltage minus the diode drop
//Vout = (Vin*R2) * (R1+R2)
//Vin = (Vout * (R1+R2))/R2
//Vin = ((((AREF/1024.0)*adDECIMAL) * (R1+R2)) / R2) + Diode drop //(aref/1024)*adDecimal is Vout
//Vout connected to Ain0 on any Arduino
/* Circuit:
  PIN57--FL_LED--150ohm--GND
  PIN58--FR_LED--150ohm--GND
  PIN59--RR_LED--150ohm--GND
  PIN60--RL_LED--150ohm--GND
*/

class BatteryMonitor_APM : public BatteryMonitor {
private:
  #define FL_LED 57 // Ain 2 on Mega
  #define FR_LED 58 // Ain 3 on Mega
  #define RR_LED 59 // Ain 4 on Mega
  #define RL_LED 60 // Ain 5 on Mega
  #define LEDDELAY 200
  float diode; //Schottky diode on APM board
  float batteryScaleFactor;

  void ledCW(void){
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

  void ledsON(void){
    digitalWrite(RL_LED, HIGH);
    digitalWrite(RR_LED, HIGH);
    digitalWrite(FR_LED, HIGH);
    digitalWrite(FL_LED, HIGH);
  };

  void ledsOFF(void){
    digitalWrite(RL_LED, LOW);
    digitalWrite(RR_LED, LOW);
    digitalWrite(FR_LED, LOW);
    digitalWrite(FL_LED, LOW);
  };

public:
  BatteryMonitor_APM() : BatteryMonitor(){}
  void initialize(void) {
    float R1   = 10050; //the SMD 10k resistor measured with DMM
    float R2   =  3260; //3k3 user mounted resistor measured with DMM
    float Aref = 3.27F; //AREF 3V3 used (solder jumper) and measured with DMM
    batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));

    diode = 0.306F; //Schottky diode on APM board, drop measured with DMM

    analogReference(EXTERNAL); //use Oilpan 3V3 AREF or if wanted, define DEFAULT here to use VCC as reference and define that voltage in BatteryReadArmLed.h

    pinMode(FL_LED ,OUTPUT);
    pinMode(FR_LED ,OUTPUT);
    pinMode(RR_LED ,OUTPUT);
    pinMode(RL_LED ,OUTPUT);

    //batteryVoltage = readBatteryVoltage(BATTERYPIN);
  }

  void lowBatteryEvent(byte level) {  // <- this logic by Jose Julio
    static byte batteryCounter=0;
    byte freq;

    if (level == OK) {
      ledsON();
      autoDescent = 0; //reset autoAscent if battery is good
    }
    else {
      batteryCounter++;
      if (level == WARNING) freq = 40;  //4 seconds wait
      else freq = 5; //0.5 second wait
      if (batteryCounter < 2) ledsOFF();  //indicate with led's everytime autoDescent kicks in
      #if defined(AltitudeHold)
        if (throttle > 1400) holdAltitude -= 0.2; //-0.2m in 2 fixed rates, one where battery < 10.8V and one where battery < 10.2V, only done if in altitude hold mode
      #else
        if (throttle > 1400) autoDescent -= 2; //will remove 2µs throttle every time led's blink in two speeds (10.8 and 10.2V) as long as there is throttle to lower
      #endif
      else if (batteryCounter < freq) ledsON();
      else batteryCounter = 0;
    }
  }

  const float readBatteryVoltage(byte channel) {
    return (analogRead(channel) * batteryScaleFactor) + diode;
  }
};

// *******************************************************************************
// ************************ AeroQuad Battery Monitor *****************************
// *******************************************************************************
class BatteryMonitor_AeroQuad : public BatteryMonitor {
private:
  #if defined (__AVR_ATmega328P__)
    #define BUZZERPIN 12
  #else
    #define BUZZERPIN 49
  #endif

  byte state, firstAlarm;
  float diode; // raw voltage goes through diode on Arduino
  float batteryScaleFactor;
  long currentBatteryTime, previousBatteryTime;

public:
  BatteryMonitor_AeroQuad() : BatteryMonitor(){}

  void initialize(void) {
    float R1   = 15000;
    float R2   =  7500;
    float Aref =     5.0;
    batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));
#ifdef AeroQuad_Mini
    diode = 0.53; // measured with DMM
#else    
    diode = 0.9; // measured with DMM
#endif    
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
    return (analogRead(channel) * batteryScaleFactor) + diode;
  }
};

#endif
