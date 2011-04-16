/*
  AeroQuad v3.0 - April 2011
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

//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer 
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

////////////////////////////////////////////////////////////////////////////////
// ARG - Accelerometer, Rate Gyro
////////////////////////////////////////////////////////////////////////////////

#ifndef _AEROQUAD_KINEMATICS_ARG_H_
#define _AEROQUAD_KINEMATICS_ARG_H_

#include <WProgram.h>
#include "Kinematics.h"
#include <Axis.h>

class Kinematics_ARG : public Kinematics {
private:
  float Kp;                   // proportional gain governs rate of convergence to accelerometer/magnetometer
  float Ki;                   // integral gain governs rate of convergence of gyroscope biases
  float halfT;                // half the sample period
  float q0, q1, q2, q3;       // quaternion elements representing the estimated orientation
  float exInt, eyInt, ezInt;  // scaled integral error

  ////////////////////////////////////////////////////////////////////////////////
  // argUpdate
  ////////////////////////////////////////////////////////////////////////////////
  void argUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, unsigned long G_Dt);
  
  void eulerAngles(void) {
    angle[ROLL]  =  atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
    angle[PITCH] =   asin(2 * (q0*q2 - q1*q3));
    angle[YAW]   =  atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
  }


public:
  Kinematics_ARG() {}
  
  ////////////////////////////////////////////////////////////////////////////////
  // Initialize ARG
  ////////////////////////////////////////////////////////////////////////////////
  void initialize(float hdgX, float hdgY);
  
  ////////////////////////////////////////////////////////////////////////////////
  // Calculate ARG
  ////////////////////////////////////////////////////////////////////////////////
  void calculate(float rollRate,          float pitchRate,    float yawRate,       \
                 float longitudinalAccel, float lateralAccel, float verticalAccel, \
                 float measuredMagX,      float measuredMagY, float measuredMagZ,  \
				 unsigned long G_Dt);
  
  float getGyroUnbias(byte axis);
};

#endif