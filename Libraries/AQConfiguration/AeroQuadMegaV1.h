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

#ifndef _AQ_MEGA_WII_CONFIG_H_
#define _AQ_MEGA_WII_CONFIG_H_


// Special thanks to Wilafau for fixes for this setup
// http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
#include <ReceiverForMega.h>
ReceiverForMega tempReceiver;
Receiver *receiver = &tempReceiver;
#include <ADXL335Accelerometer.h>
ADXL335Accelerometer tempAccel;
Accelerometer *accel = &tempAccel;
#include <IDGIXZ500Gyroscope.h>
IDGIXZ500Gyroscope tempGyro;
Gyroscope *gyro = &tempGyro;
#include <PWMMotors.h>
PWMMotors tempMotors;
Motors *motors = &tempMotors;
#include <FlightAngleDCM.h>
FlightAngleDCM tempFlightAngle;
FlightAngleProcessor *flightAngle = &tempFlightAngle;
#ifdef CameraControl
  #include <AeroQuadCameraStabilizer.h>
  AeroQuadCameraStabilizer tempCamera;
  CameraStabilizer *cameraStabilizer = &tempCamera;
#endif

void initPlatform()
{
}

#endif