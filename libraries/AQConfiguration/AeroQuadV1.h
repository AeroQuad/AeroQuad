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

#ifndef _AQ_V1_CONFIG_H_
#define _AQ_V1_CONFIG_H_

#define AeroQuad_v1

#include <ADXL335Accelerometer.h>
ADXL335Accelerometer tempAccel;
Accelerometer *_accel = &tempAccel;
#include <IDGIXZ500Gyroscope.h>
IDGIXZ500Gyroscope tempGyro;
Gyroscope *_gyro = &tempGyro;
#include <ReceiverFor328p.h>
ReceiverFor328p tempReceiver;
Receiver *_receiver = &tempReceiver;
#include <PWMMotors.h>
PWMMotors tempMotors;
Motors *_motors = &tempMotors;
//#include "FlightAngle.h"
//FlightAngleDCM tempFlightAngle;
//FlightAngleProcessor *_flightAngle = &tempFlightAngle;
//  #ifdef CameraControl
//    #include "AeroQuadCameraStabilizer.h"
//    AeroQuadCameraStabilizer tempCamera;
//    CameraStabilizer *_cameraStabilizer = &tempCamera;
//  #endif
//#endif

#endif