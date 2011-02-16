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

#ifndef _AQ_FLIGHT_ANGLE_PROCESSOR_IMU_FILTER_H_
#define _AQ_FLIGHT_ANGLE_PROCESSOR_IMU_FILTER_H_

#include "FlightAngleProcessor.h"

class FlightAngleIMU : public FlightAngleProcessor 
{
private:
  // System constants
  #define gyroMeasError 3.14159265358979f * (75.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
  #define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
  float SEq_1, SEq_2, SEq_3, SEq_4; // estimated orientation quaternion elements with initial conditions

  void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z,float G_Dt);
  
public:
  FlightAngleIMU() {}
  
  void initialize(void);
  
  void calculate(float G_Dt);
  
  float getGyroUnbias(byte axis);

  void calibrate(void) {}
};

#endif