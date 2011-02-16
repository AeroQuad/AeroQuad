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

#ifndef _AQ_APM_OP_CHR6DM_CONFIG_H_
#define _AQ_APM_OP_CHR6DM_CONFIG_H_

#define LED_Red 35
#define LED_Yellow 36
#define LED_Green 37
#define RELE_pin 47
#define SW1_pin 41
#define SW2_pin 40
#define BUZZER 9
#define PIANO_SW1 42
#define PIANO_SW2 43


//#include <CHR6DMSensorsAccessor.h>
CHR6DM chr6dm;
#include <CHR6DMAccelerometer.h>
CHR6DMAccelerometer tempAccel;
Accelerometer *accel = &tempAccel;
#include <CHR6DMGyroscope.h>
CHR6DMGyroscope tempGyro;
Gyroscope *gyro = &tempGyro;
#include <ReceiverForAPM.h>
ReceiverForAPM tempReceiver;
Receiver *receiver = &tempReceiver;
#include <APMMotors.h>
APMMotors tempMotors;
Motors *motors = &tempMotors;
#include "FlightAngleDCM.h"
//  FlightAngleCHR6DM tempFlightAngle;
FlightAngleDCM tempFlightAngle;
FlightAngleProcessor *flightAngle = &tempFlightAngle;
#include "CHR6DMCompass.h"
CHR6DMCompass tempCompass;
Compass *compass = &tempCompass;
#ifdef AltitudeHold
  #include <BMP085BarometricSensor.h>
  BMP085BarometricSensor tempAltitude;
  AltitudeProvider *altitudeProvider = &tempAltitude;
#endif
#ifdef BattMonitor
  #include "APMBatteryMonitor.h"
  APMBatteryMonitor tempBatteryMonitor;
  BatteryMonitor *batteryMonitor = &tempBatteryMonitor;
#endif
#ifdef CameraControl
  #include <AeroQuadCameraStabilizer.h>
  AeroQuadCameraStabilizer tempCamera;
  CameraStabilizer *cameraStabilizer = &tempCamera;
#endif


void initPlatform()
{
  Serial1.begin(BAUD);
  PORTD = B00000100;
  
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Yellow, OUTPUT);
  pinMode(LED_Green, OUTPUT);

  Wire.begin();
  
  tempAccel.setChr6dm(&chr6dm);
  tempGyro.setChr6dm(&chr6dm);
  tempCompass.setChr6dm(&chr6dm);
}


#endif
