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


/******************************************************
*************** SAMPLE OF CUSTOM CONFIG ***************
******************* DON'T USE *************************
******************************************************/
#ifndef _AQ_CURSOM_CONFIG_H_
#define _AQ_CURSOM_CONFIG_H_

#include <ADXL335Accelerometer.h>
ADXL335Accelerometer tempAccel;
Accelerometer *_accel = &tempAccel;
#include <ITG3200Gyroscope.h>
ITG3200Gyroscope tempGyro;
Gyroscope *_gyro = &tempGyro;
#include <ReceiverFor328p.h>
ReceiverFor328p tempReceiver;
Receiver *_receiver = &tempReceiver;
#include <I2CMotors.h>
I2CMotors tempMotors;
Motors *_motors = &tempMotors;
//Motors_AeroQuadI2C motors; // Use for I2C based ESC's
#include "FlightAngleDCM.h"
FlightAngleDCM tempFlightAngle;
//  #include "FlightAngleCompFilter.h"
//  FlightAngleCompFilter tempFlightAngle;
//  #include "FlightAngleKalmanFilter.h"
//  FlightAngleKalmanFilter tempFlightAngle;
FlightAngleProcessor *_flightAngle = &tempFlightAngle;
#ifdef HeadingMagHold
  #include <HMC5843Magnetometer.h>
  HMC5843Magnetometer tempCompass(_gyro);
  Compass *_compass = &tempCompass;
#endif
#ifdef AltitudeHold
  #include <BMP085BarometricSensor.h>
  BMP085BarometricSensor tempAltitude;
  AltitudeProvider *_altitudeProvider = &tempAltitude;
#endif
#ifdef BattMonitor
  #include <AeroQuadBatteryMonitor.h>
  AeroQuadBatteryMonitor tempBatteryMonitor;
  BatteryMonitor *_batteryMonitor = &tempBatteryMonitor;
#endif


#endif
