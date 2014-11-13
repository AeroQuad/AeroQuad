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


double exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error
double Kp, Ki;  

////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void calculateKinematicsAGR(double gx, double gy, double gz, double ax, double ay, double az, double G_Dt) {
  
    
  halfT = G_Dt/2;
  
  // normalise the measurements
  double norm = sqrt(ax*ax + ay*ay + az*az);       
  calculateAccConfidence(norm);
  Kp = DEFAULT_Kp * accConfidence;
  Ki = DEFAULT_Ki * accConfidence;
	
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
     	
  // estimated direction of gravity and flux (v and w)
  double vx = 2*(q1*q3 - q0*q2);
  double vy = 2*(q0*q1 + q2*q3);
  double vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  double ex = (vy*az - vz*ay);
  double ey = (vz*ax - vx*az);
  double ez = (vx*ay - vy*ax);
    
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
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
    
  // integrate quaternion rate and normalise
  q0 += ((-q1*gx - q2*gy - q3*gz) * halfT);
  q1 += (( q0*gx + q2*gz - q3*gy) * halfT);
  q2 += (( q0*gy - q1*gz + q3*gx) * halfT);
  q3 += (( q0*gz + q1*gy - q2*gx) * halfT);
    
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
  // eulerAngles
  kinematicsAngle[XAXIS] = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  kinematicsAngle[ZAXIS] = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}
  
////////////////////////////////////////////////////////////////////////////////
// Initialize ARG
////////////////////////////////////////////////////////////////////////////////

void initializeKinematics() 
{
  initializeBaseKinematicParam();
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
}
  
 
#endif

