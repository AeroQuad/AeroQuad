/*
  AeroQuad v2.1 - November 2010
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

#define LOW_VOLTAGE_1 10.8    // Pack voltage at which to trigger alarm (first alarm)
#define LOW_VOLTAGE_2 10.2    // Pack voltage at which to trigger alarm (critical alarm)
#define BATTERYPIN 0          // Ain 0 (universal to every Arduino), pin 55 on Mega (1280)

#define R1 10050 //the SMD 10k resistor measured with DMM
#define R2 3260 //3k3 user mounted resistor measured with DMM
#define AREF 3.27F //AREF 3V3 used (solder jumper) and measured with DMM
#define DIODE_FWD_VOLTAGE_DROP 0.306F //Schottky diode on APM board, drop measured with DMM. If no diode present, define as 0

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

#define FL_LED 57 // Ain 2 on Mega
#define FR_LED 58 // Ain 3 on Mega
#define RR_LED 59 // Ain 4 on Mega
#define RL_LED 60 // Ain 5 on Mega
#define LEDDELAY 200

  /* Circuit:
  PIN57--FL_LED--150ohm--GND
  PIN58--FR_LED--150ohm--GND
  PIN59--RR_LED--150ohm--GND
  PIN60--RL_LED--150ohm--GND
  */

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


void lowBatteryEvent(byte level) // <- this logic by Jose Julio
{
  byte batteryCounter=0;
  byte freq;

  if (level==0) {
    ledsON();
    autoDescent = 0; //reset autoAscent if battery is good
  }
  else 
    {
     batteryCounter++;
    if (level==1)
      freq = 40;  //4 seconds wait
    else 
      freq = 5; //0.5 second wait
    if (batteryCounter<2)
         ledsOFF();          //indicate with led's everytime autoDescent kicks in
         
         #ifndef AltitudeHold
         #ifdef AutoDescent
         if(throttle > 1400) { autoDescent -= 2; }//will remove 2µs throttle every time led's blink in two speeds (10.8 and 10.2V) as long as there is throttle to lower
         #endif
         #endif
         #if defined(AltitudeHold) && defined(AutoDescent)
         if(throttle > 1400) { holdAltitude -= 0.2; } //-0.2m in 2 fixed rates, one where battery < 10.8V and one where battery < 10.2V, only done if in altitude hold mode
         #endif
         
    else if(batteryCounter < freq)
          ledsON();
    else
      batteryCounter=0;
    }
}

float readBattery(void)
{   
  byte batteryStatus = 0;
  
  float batteryVoltage = (((analogRead(BATTERYPIN)*(AREF/1024.0)) * (R1+R2)) / R2) + DIODE_FWD_VOLTAGE_DROP; //max 13.5V! Honk gets 0.01V difference from this function compared to DMM
  //Serial.println(battery_voltage); //this is accessed by typing '=' in Serial Monitor
  
  if (batteryStatus==2)   // If we reach battery status 2, we mantain this status of alert
    batteryStatus=2;  
  else if (batteryVoltage < LOW_VOLTAGE_2)
    batteryStatus=2;
  else if (batteryVoltage < LOW_VOLTAGE_1)
    batteryStatus=1;
  else
    batteryStatus=0;
  lowBatteryEvent(batteryStatus);
  
  return batteryVoltage;
}
