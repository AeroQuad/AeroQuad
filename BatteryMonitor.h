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

  virtual void initialize(void);
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



#define FL_LED 57 // Ain 2 on Mega
#define FR_LED 58 // Ain 3 on Mega
#define RR_LED 59 // Ain 4 on Mega
#define RL_LED 60 // Ain 5 on Mega
#define LEDDELAY 200

class BatteryMonitor_APM : public BatteryMonitor 
{
private:
  float _diode; //Schottky diode on APM board
  float _batteryScaleFactor;

  void ledCW(void)
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
    _batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));

    _diode = 0.306F; //Schottky diode on APM board, drop measured with DMM

    analogReference(EXTERNAL); //use Oilpan 3V3 AREF or if wanted, define DEFAULT here to use VCC as reference and define that voltage in BatteryReadArmLed.h

    pinMode(FL_LED ,OUTPUT);
    pinMode(FR_LED ,OUTPUT);
    pinMode(RR_LED ,OUTPUT);
    pinMode(RL_LED ,OUTPUT);

    //batteryVoltage = readBatteryVoltage(BATTERYPIN);
  }

  void lowBatteryEvent(byte level) // <- this logic by Jose Julio
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
        if (_throttle > 1400) 
        {
          _holdAltitude -= 0.2; //-0.2m in 2 fixed rates, one where battery < 10.8V and one where battery < 10.2V, only done if in altitude hold mode
        }
      #else
        if (_throttle > 1400) 
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

  const float readBatteryVoltage(byte channel) 
  {
    return (analogRead(channel) * _batteryScaleFactor) + _diode;
  }
};

// *******************************************************************************
// ************************ AeroQuad Battery Monitor *****************************
// *******************************************************************************
class BatteryMonitor_AeroQuad : public BatteryMonitor 
{
private:
  #if defined (__AVR_ATmega328P__)
    #define BUZZERPIN 12
  #else
    #define BUZZERPIN 49
  #endif
  
  long _previousTime;
  byte _state;
  byte _firstAlarm;
  float _diode; // raw voltage goes through diode on Arduino
  float _batteryScaleFactor;

public:
  BatteryMonitor_AeroQuad() : BatteryMonitor(){}

  void initialize(void) 
  {
    float R1   = 15000;
    float R2   =  7500;
    float Aref =     5.0;
    _batteryScaleFactor = ((Aref / 1024.0) * ((R1 + R2) / R2));    
    _diode = 0.9; // measured with DMM
    analogReference(DEFAULT);
    pinMode(BUZZERPIN, OUTPUT); // connect a 12V buzzer to pin 49
    digitalWrite(BUZZERPIN, LOW);
    _previousTime = millis();
    _state = LOW;
    _firstAlarm = OFF;
  }

  void lowBatteryEvent(byte level) 
  {
    long currentTime = millis()- _previousTime;
    if (level == OK) 
    {
      digitalWrite(BUZZERPIN, LOW);
      _autoDescent = 0;
    }
    if (level == WARNING) 
    {
      if ((_autoDescent == 0) && (_currentTime > 1000)) 
      {
        _autoDescent = -50;
      }
      if (_currentTime > 1100) 
      {
        _autoDescent = 50;
        digitalWrite(LED2PIN, HIGH);
        digitalWrite(BUZZERPIN, HIGH);
      }
      if (_currentTime > 1200) 
      {
        _previousTime = millis();
        _autoDescent = 0;
        digitalWrite(LED2PIN, LOW);
        digitalWrite(BUZZERPIN, LOW);
      }
    }
    if (level == ALARM) 
    {
      if (_firstAlarm == OFF) 
      {
        _autoDescent = 0; // intialize autoDescent to zero if first time in ALARM state
      }
      _firstAlarm = ON;
      digitalWrite(BUZZERPIN, HIGH); // enable buzzer
      if ((_currentTime > 500) && (_throttle > 1400)) 
      {
        _autoDescent -= 1; // auto descend quad
        _holdAltitude -= 0.2; // descend if in attitude hold mode
        _previousTime = millis();
        if (_state == LOW) 
        {
          _state = HIGH;
        }
        else
        { 
          _state = LOW;
        }
        digitalWrite(LEDPIN, _state);
        digitalWrite(LED2PIN, _state);
        digitalWrite(LED3PIN, _state);
      }
    }
  }

  const float readBatteryVoltage(byte channel) 
  {
    return (analogRead(channel) * _batteryScaleFactor) + _diode;
  }
};

