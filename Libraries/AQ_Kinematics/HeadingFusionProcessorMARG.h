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

#ifndef _AQ_HEADING_FURSION_PROCESSOR_ARG_BASE_
#define _AQ_HEADING_FURSION_PROCESSOR_ARG_BASE_

#include <AQMath.h>


#if defined UseGPS
  #include "MagnetometerDeclinationDB.h"
#endif  

#include <Compass.h>

float trueNorthHeading = 0.0;
float compassDeclination = 0.0;

float headingAngle[3] = {0.0,0.0,0.0};

float lkpMag = 0.0;                					// proportional gain governs rate of convergence to magnetometer
float lkiMag = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float lhalfT = 0.0;                					// half the sample period
float lq0 = 0.0, lq1 = 0.0, lq2 = 0.0, lq3 = 0.0;       // quaternion elements representing the estimated orientation
float lexInt = 0.0, leyInt = 0.0, lezInt = 0.0;  		// scaled integral error
  
////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void headingUpdate(float gx, float gy, float gz, float mx, float my, float mz, float G_Dt) {
  
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, wx, wy;//, wz;
  float q0i, q1i, q2i, q3i;
  float exAcc, eyAcc, ezAcc;
  float ezMag;
    
  lhalfT = G_Dt/2;
    
  // normalise the measurements
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;         

  // compute reference direction of flux
  hx = mx * 2*(0.5 - lq2*lq2 - lq3*lq3) + my * 2*(lq1*lq2 - lq0*lq3)       + mz * 2*(lq1*lq3 + lq0*lq2);
  hy = mx * 2*(lq1*lq2 + lq0*lq3)       + my * 2*(0.5 - lq1*lq1 - lq3*lq3) + mz * 2*(lq2*lq3 - lq0*lq1);
  hz = mx * 2*(lq1*lq3 - lq0*lq2)       + my * 2*(lq2*lq3 + lq0*lq1)       + mz * 2*(0.5 - lq1*lq1 - lq2*lq2);
    
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(lq1*lq3 - lq0*lq2);
  vy = 2*(lq0*lq1 + lq2*lq3);
      
  wx = bx * 2*(0.5 - lq2*lq2 - lq3*lq3) + bz * 2*(lq1*lq3 - lq0*lq2);
  wy = bx * 2*(lq1*lq2 - lq0*lq3)       + bz * 2*(lq0*lq1 + lq2*lq3);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  exAcc = (vy*-9.8);
  eyAcc = (-vx*-9.8);
  ezAcc = 0.0;
    
  ezMag = (mx*wy - my*wx);
    
  // integral error scaled integral gain
  lexInt += exAcc;
  leyInt += eyAcc;
  lezInt += ezAcc;
	
  // adjusted gyroscope measurements
  gx = gx + exAcc + lexInt;
  gy = gy + eyAcc + leyInt;
  gz = gz + ezAcc + ezMag*lkpMag + lezInt;

  
  // integrate quaternion rate and normalise
  q0i = (-lq1*gx - lq2*gy - lq3*gz) * lhalfT;
  q1i = ( lq0*gx + lq2*gz - lq3*gy) * lhalfT;
  q2i = ( lq0*gy - lq1*gz + lq3*gx) * lhalfT;
  q3i = ( lq0*gz + lq1*gy - lq2*gx) * lhalfT;
  lq0 += q0i;
  lq1 += q1i;
  lq2 += q2i;
  lq3 += q3i;
    
  // normalise quaternion
  norm = sqrt(lq0*lq0 + lq1*lq1 + lq2*lq2 + lq3*lq3);
  lq0 = lq0 / norm;
  lq1 = lq1 / norm;
  lq2 = lq2 / norm;
  lq3 = lq3 / norm;
}
  
void headingEulerAngles()
{
  headingAngle[XAXIS] = atan2(2 * (lq0*lq1 + lq2*lq3), 1 - 2 *(lq1*lq1 + lq2*lq2));
  headingAngle[YAXIS] = asin(2 * (lq0*lq2 - lq1*lq3));
  headingAngle[ZAXIS] = atan2(2 * (lq0*lq3 + lq1*lq2), 1 - 2 *(lq2*lq2 + lq3*lq3));
}

void initializeHeadingFusion()  
{
  float yawAngle = atan2(hdgY, hdgX);

  headingAngle[XAXIS] = 0.0;
  headingAngle[YAXIS] = 0.0;
  headingAngle[ZAXIS] = yawAngle;

  lq0 = cos(0.0)*cos(0.0)*cos(yawAngle/2) + sin(0.0)*sin(0.0)*sin(yawAngle/2);
  lq1 = sin(0.0)*cos(0.0)*cos(yawAngle/2) - cos(0.0)*sin(0.0)*sin(yawAngle/2);
  lq2 = cos(0.0)*sin(0.0)*cos(yawAngle/2) + sin(0.0)*cos(0.0)*sin(yawAngle/2);
  lq3 = cos(0.0)*cos(0.0)*sin(yawAngle/2) - sin(0.0)*sin(0.0)*cos(yawAngle/2);

  lkpMag = 0.2;//2.0;
  lkiMag = 0.0005;//0.005;
}


////////////////////////////////////////////////////////////////////////////////
// Calculate ARG
////////////////////////////////////////////////////////////////////////////////
void localCalculateHeading(float rollRate,          float pitchRate,    float yawRate,  
                           float measuredMagX,      float measuredMagY, float measuredMagZ,
				           float G_Dt) {
    
  headingUpdate(rollRate,          pitchRate,    yawRate, 
             measuredMagX,      measuredMagY, measuredMagZ,
		     G_Dt);
  headingEulerAngles();
  trueNorthHeading = headingAngle[ZAXIS];
  
  #if defined UseGPS
    if( compassDeclination != 0.0 ) {

	  trueNorthHeading = trueNorthHeading + compassDeclination;
	  if (trueNorthHeading > M_PI)  {  // Angle normalization (-180 deg, 180 deg)
	    trueNorthHeading -= (2.0 * M_PI);
	  } 
	  else if (trueNorthHeading < -M_PI){
	    trueNorthHeading += (2.0 * M_PI);
	  }
    }
  #endif
}

void calculateHeading()
{
  localCalculateHeading(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], 
                     measuredMag[XAXIS], measuredMag[YAXIS], measuredMag[ZAXIS],
                     G_Dt);					 
					 
}
  
  
#if defined UseGPS
  void setDeclinationLocation(long lat, long lon) {
    // get declination ( in radians )
    compassDeclination = getMagnetometerDeclination(lat, lon);    
  }
#endif


#endif

