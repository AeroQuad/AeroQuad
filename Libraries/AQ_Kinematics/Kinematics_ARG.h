/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_KINEMATICS_ARG_
#define _AQ_KINEMATICS_ARG_

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


#include "Kinematics.h"

#include <AQMath.h>

float Kp = 0.0;                   					// proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki = 0.0;                   					// integral gain governs rate of convergence of gyroscope biases
float halfT = 0.0;                					// half the sample period
float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error
  
float previousEx = 0.0;
float previousEy = 0.0;
float previousEz = 0.0;

float ax = 0.0;
float ay = 0.0;
float az = 0.0;

float gx = 0.0;
float gy = 0.0;
float gz = 0.0;


////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void argUpdate(float l_gx, float l_gy, float l_gz, float l_ax, float l_ay, float l_az, float G_Dt) {
  
  float norm;
  float vx, vy, vz;
  float q0i, q1i, q2i, q3i;
  float ex, ey, ez;
    
  halfT = G_Dt/2;
  
  // normalise the measurements
  norm = sqrt(l_ax*l_ax + l_ay*l_ay + l_az*l_az);       
  ax = l_ax / norm;
  ay = l_ay / norm;
  az = l_az / norm;
     	
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (vy*az - vz*ay);
  ey = (vz*ax - vx*az);
  ez = (vx*ay - vy*ax);
    
  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
	
  eyInt = eyInt + ey*Ki;
  if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;

  ezInt = ezInt + ez*Ki;
  if (isSwitched(previousEz,ez)) {
    ezInt = 0.0;
  }
  previousEz = ez;
	
  // adjusted gyroscope measurements
  gx = l_gx + Kp*ex + exInt;
  gy = l_gy + Kp*ey + eyInt;
  gz = l_gz + Kp*ez + ezInt;
    
  // integrate quaternion rate and normalise
  q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
  q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
  q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
  q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
  q0 += q0i;
  q1 += q1i;
  q2 += q2i;
  q3 += q3i;
    
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
}
  
void eulerAngles()
{
  kinematicsAngle[XAXIS]  = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  kinematicsAngle[ZAXIS]   = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

////////////////////////////////////////////////////////////////////////////////
// Initialize ARG
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics(float hdgX, float hdgY) 
{
  initializeBaseKinematicsParam(hdgX,hdgY);
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
	
  previousEx = 0;
  previousEy = 0;
  previousEz = 0;

  Kp = 0.2; // 2.0;
  Ki = 0.0005; //0.005;
}
  
////////////////////////////////////////////////////////////////////////////////
// Calculate ARG
////////////////////////////////////////////////////////////////////////////////
void calculateKinematics(float rollRate,          float pitchRate,    float yawRate,  
                         float longitudinalAccel, float lateralAccel, float verticalAccel, 
                         float G_DT) {
    
  argUpdate(rollRate,          pitchRate,    yawRate, 
            longitudinalAccel, lateralAccel, verticalAccel,  
		    G_Dt);
  eulerAngles();
}
  
float getGyroUnbias(byte axis) {
  return correctedRateVector[axis];
}
  
void calibrateKinematics() {}


#endif

