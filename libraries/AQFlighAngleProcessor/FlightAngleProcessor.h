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

#ifndef _AQ_FLIGHT_ANGLE_PROCESSOR_H_
#define _AQ_FLIGHT_ANGLE_PROCESSOR_H_

#define CF 0
#define KF 1
#define DCM 2
#define IMU 3

#include <Gyroscope.h>
#include <Accelerometer.h>

// This class is responsible for calculating vehicle attitude
class FlightAngleProcessor 
{
private:
  float _gyroAngle[2];

protected:
  byte _type;
  float _angle[3];
  
  Gyroscope *_gyro;
  Accelerometer *_accel;

public:
  
  FlightAngleProcessor(void);
  
  virtual void initialize();
  virtual void calculate(float G_Dt);
  virtual float getGyroUnbias(byte axis);
  virtual void calibrate();
 
  const float getData(byte axis);
  
  const byte getType(void);
  
  void setGyroscope(Gyroscope *gyro);
  void setAccelerometer(Accelerometer *accel);
};

#endif