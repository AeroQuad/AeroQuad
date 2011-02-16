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

#ifndef _AQ_V18_CONFIG_H_
#define _AQ_V18_CONFIG_H_


#include <BMA180Accelerometer.h>
BMA180Accelerometer tempAccel;
Accelerometer *accel = &tempAccel;
#include <ITG3200Gyroscope.h>
ITG3200Gyroscope tempGyro;
Gyroscope *gyro = &tempGyro;
#include <ReceiverFor328p.h>
ReceiverFor328p tempReceiver;
Receiver *receiver = &tempReceiver;
#include <PWMTimedMotors.h>
PWMTimedMotors tempMotors;
Motors *motors = &tempMotors;
//Motors_AeroQuadI2C motors; // Use for I2C based ESC's
//#include "FlightAngleDCM.h"
//FlightAngleDCM tempFlightAngle;
//#include "FlightAngleCompFilter.h"
//FlightAngleCompFilter tempFlightAngle;
#include "FlightAngleKalmanFilter.h"
FlightAngleKalmanFilter tempFlightAngle;
FlightAngleProcessor *flightAngle = &tempFlightAngle;
#ifdef HeadingMagHold
  #include <HMC5843Magnetometer.h>
  HMC5843Magnetometer tempCompass;
  Compass *compass = &tempCompass;
#endif
#ifdef AltitudeHold
#include <BMP085BarometricSensor.h>
  BMP085BarometricSensor tempAltitude;
  AltitudeProvider *_altitudeProvider = &tempAltitude;
#endif
#ifdef BattMonitor
#include <AeroQuadBatteryMonitor.h>
  AeroQuadBatteryMonitor tempBatteryMonitor;
  BatteryMonitor *batteryMonitor = &tempBatteryMonitor;
#endif
//  #ifdef CameraControl
//    #include "AeroQuadCameraStabilizer.h"
//    AeroQuadCameraStabilizer tempCamera;
//    CameraStabilizer *_cameraStabilizer = &tempCamera;
//  #endif

void initPlatform()
{
  pinMode(LED2PIN, OUTPUT);
  digitalWrite(LED2PIN, LOW);
  pinMode(LED3PIN, OUTPUT);
  digitalWrite(LED3PIN, LOW);
  
  Wire.begin();  
  TWBR = 12;
  
  #ifdef HeadingMagHold
    tempCompass.setGyroscope(gyro);
  #endif

}
#endif