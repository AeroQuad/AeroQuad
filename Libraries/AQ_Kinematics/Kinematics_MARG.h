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
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================

////////////////////////////////////////////////////////////////////////////////
// MARG - Magntometer, Accelerometer, Rate Gyro
////////////////////////////////////////////////////////////////////////////////

#ifndef _AEROQUAD_KINEMATICS_MARG_H_
#define _AEROQUAD_KINEMATICS_MARG_H_

#include <WProgram.h>
#include "Kinematics.h"
#include "Axis.h"

class Kinematics_MARG : public Kinematics {
private:
  float kpAcc;                // proportional gain governs rate of convergence to accelerometer
  float kiAcc;                // integral gain governs rate of convergence of gyroscope biases
  float kpMag;                // proportional gain governs rate of convergence to magnetometer
  float kiMag;                // integral gain governs rate of convergence of gyroscope biases
  float halfT;                // half the sample period
  float q0, q1, q2, q3;       // quaternion elements representing the estimated orientation
  float exInt, eyInt, ezInt;  // scaled integral error

  ////////////////////////////////////////////////////////////////////////////////
  // margUpdate
  ////////////////////////////////////////////////////////////////////////////////
  void margUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, unsigned long G_Dt);
  
  void eulerAngles(void) {
    angle[ROLL]  =  atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
    angle[PITCH] =   asin(2 * (q0*q2 - q1*q3));
    angle[YAW]   =  atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
  }


public:
  Kinematics_MARG() {}
  
  ////////////////////////////////////////////////////////////////////////////////
  // Initialize MARG
  ////////////////////////////////////////////////////////////////////////////////
  void initialize(float hdgX, float hdgY);
  
  ////////////////////////////////////////////////////////////////////////////////
  // Calculate MARG
  ////////////////////////////////////////////////////////////////////////////////

  void calculate(float rollRate,          float pitchRate,    float yawRate,  \
                 float longitudinalAccel, float lateralAccel, float verticalAccel, \
                 float measuredMagX,      float measuredMagY, float measuredMagZ,
				 unsigned long G_Dt);
  
  float getGyroUnbias(byte axis);
};

#endif