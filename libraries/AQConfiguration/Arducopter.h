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

#ifndef _AQ_ARDUCOPTER_CONFIG_H_
#define _AQ_ARDUCOPTER_CONFIG_H_

#define LED_Red 35
#define LED_Yellow 36
#define LED_Green 37
#define RELE_pin 47
#define SW1_pin 41
#define SW2_pin 40
#define BUZZER 9
#define PIANO_SW1 42
#define PIANO_SW2 43


#include <IDG500_ADCGyroscope.h>
IDG500_ADCGyroscope tempGyro;
Gyroscope *gyro = &tempGyro;
#include <ADXL335_ADCAccelerometer.h>
ADXL335_ADCAccelerometer tempAccel;
Accelerometer *accel = &tempAccel;
#include <ReceiverForAPM.h>
ReceiverForAPM tempReceiver;
Receiver *receiver = &tempReceiver;
#include <APMMotors.h>
APMMotors tempMotors;
Motors *motors = &tempMotors;
#include "FlightAngleDCM.h"
FlightAngleDCM tempFlightAngle;
FlightAngleProcessor *flightAngle = &tempFlightAngle;
#ifdef HeadingMagHold
//  #include <HMC5843Magnetometer.h>
//  HMC5843Magnetometer tempCompass;
  #include <HMC6352Magnetometer.h>
  HMC6352Magnetometer tempCompass;
  Compass *compass = &tempCompass;
#endif
#ifdef AltitudeHold
//  #include <BMP085BarometricSensor.h>
//  BMP085BarometricSensor tempAltitude;
  #include <MPXBarometerOilpan.h>
  MPXBarometerOilpan tempAltitude;
  AltitudeProvider *altitudeProvider = &tempAltitude;
#endif
#ifdef BattMonitor
  #include <APMBatteryMonitor.h>
  APMBatteryMonitor tempBatteryMonitor;
  BatteryMonitor *batteryMonitor = &tempBatteryMonitor;
#endif
//#endif



void initPlatform()
{
  Wire.begin();
  
  initializeADC(); // this is needed for both gyros and accels, done once in this class
  
  // needed for HMC5843Magnetometer
//  #ifdef HeadingMagHold
//    tempCompass.setGyroscope(gyro);
//  #endif
}


#endif