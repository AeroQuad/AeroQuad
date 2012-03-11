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

#ifndef _AQ_KINEMATICS_MARG_
#define _AQ_KINEMATICS_MARG_

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

#include "Kinematics.h"

float kpAcc = 0.0;                					// proportional gain governs rate of convergence to accelerometer
float kiAcc = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float kpMag = 0.0;                					// proportional gain governs rate of convergence to magnetometer
float kiMag = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float halfT = 0.0;                					// half the sample period
float q0 = 0.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;       // quaternion elements representing the estimated orientation
float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;  		// scaled integral error

////////////////////////////////////////////////////////////////////////////////
// margUpdate
////////////////////////////////////////////////////////////////////////////////

void margUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float q0i, q1i, q2i, q3i;
  float exAcc, eyAcc, ezAcc;
  float exMag, eyMag, ezMag;
    
  halfT = G_Dt/2;
  
  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
    	
  // normalise the measurements
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;         
    	
  // compute reference direction of flux
  hx = mx * 2*(0.5 - q2q2 - q3q3) + my * 2*(q1q2 - q0q3)       + mz * 2*(q1q3 + q0q2);
  hy = mx * 2*(q1q2 + q0q3)       + my * 2*(0.5 - q1q1 - q3q3) + mz * 2*(q2q3 - q0q1);
  hz = mx * 2*(q1q3 - q0q2)       + my * 2*(q2q3 + q0q1)       + mz * 2*(0.5 - q1q1 - q2q2);
    
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;        
    	
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
    
  wx = bx * 2*(0.5 - q2q2 - q3q3) + bz * 2*(q1q3 - q0q2);
  wy = bx * 2*(q1q2 - q0q3)       + bz * 2*(q0q1 + q2q3);
  wz = bx * 2*(q0q2 + q1q3)       + bz * 2*(0.5 - q1q1 - q2q2);
    	
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  exAcc = (vy*az - vz*ay);
  eyAcc = (vz*ax - vx*az);
  ezAcc = (vx*ay - vy*ax);
    
  exMag = (my*wz - mz*wy);
  eyMag = (mz*wx - mx*wz);
  ezMag = (mx*wy - my*wx);
    
  // integral error scaled integral gain
  exInt = exInt + exAcc*kiAcc + exMag*kiMag;
  eyInt = eyInt + eyAcc*kiAcc + eyMag*kiMag;
  ezInt = ezInt + ezAcc*kiAcc + ezMag*kiMag;
    	
  // adjusted gyroscope measurements
  correctedRateVector[XAXIS] = gx = gx + exAcc*kpAcc + exMag*kpMag + exInt;
  correctedRateVector[YAXIS] = gy = gy + eyAcc*kpAcc + eyMag*kpMag + eyInt;
  correctedRateVector[ZAXIS] = gz = gz + ezAcc*kpAcc + ezMag*kpMag + ezInt;
    	
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
    
  // save the adjusted gyroscope measurements
  correctedRateVector[XAXIS] = gx;
  correctedRateVector[YAXIS] = gy;
  correctedRateVector[ZAXIS] = gz;
}
  
void eulerAngles(void)
{
  kinematicsAngle[XAXIS]  = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  kinematicsAngle[YAXIS] = asin(2 * (q0*q2 - q1*q3));
  kinematicsAngle[ZAXIS]   = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

  
////////////////////////////////////////////////////////////////////////////////
// Initialize MARG
////////////////////////////////////////////////////////////////////////////////
void initializeKinematics(float hdgX, float hdgY) 
{
  initializeBaseKinematicsParam(hdgX,hdgY);
  float hdg = atan2(hdgY, hdgX);
    
  q0 = cos(hdg/2);
  q1 = 0;
  q2 = 0;
  q3 = sin(hdg/2);
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;

  kpAcc = 0.2;
  kiAcc = 0.0005;
    
  kpMag = 2.0;
  kiMag = 0.005;
}
  
////////////////////////////////////////////////////////////////////////////////
// Calculate MARG
////////////////////////////////////////////////////////////////////////////////

void calculateKinematics(float rollRate,          float pitchRate,    float yawRate,  
                 float longitudinalAccel, float lateralAccel, float verticalAccel, 
                 float measuredMagX,      float measuredMagY, float measuredMagZ,
				 float G_Dt) {
    
  margUpdate(rollRate,          pitchRate,    yawRate, 
             longitudinalAccel, lateralAccel, verticalAccel,  
             measuredMagX,      measuredMagY, measuredMagZ,
		     G_Dt);
  eulerAngles();
}
  
float getGyroUnbias(byte axis) {
  return correctedRateVector[axis];
}
  
void calibrateKinematics() {}


#endif

