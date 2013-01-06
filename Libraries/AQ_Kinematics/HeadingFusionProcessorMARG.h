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

float lkpAcc = 0.0;                					// proportional gain governs rate of convergence to accelerometer
float lkiAcc = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float lkpMag = 0.0;                					// proportional gain governs rate of convergence to magnetometer
float lkiMag = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float lhalfT = 0.0;                					// half the sample period
float lq0 = 0.0, lq1 = 0.0, lq2 = 0.0, lq3 = 0.0;       // quaternion elements representing the estimated orientation
float lexInt = 0.0, leyInt = 0.0, lezInt = 0.0;  		// scaled integral error
  

////////////////////////////////////////////////////////////////////////////////
// argUpdate
////////////////////////////////////////////////////////////////////////////////
void headingUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float G_Dt) {
  
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy;
  float q0i, q1i, q2i, q3i;
  float exAcc, eyAcc, ezAcc;
  float ezMag;
    
  lhalfT = G_Dt/2;
    
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
  hx = mx * 2*(0.5 - lq2*lq2 - lq3*lq3) + my * 2*(lq1*lq2 - lq0*lq3)       + mz * 2*(lq1*lq3 + lq0*lq2);
  hy = mx * 2*(lq1*lq2 + lq0*lq3)       + my * 2*(0.5 - lq1*lq1 - lq3*lq3) + mz * 2*(lq2*lq3 - lq0*lq1);
  hz = mx * 2*(lq1*lq3 - lq0*lq2)       + my * 2*(lq2*lq3 + lq0*lq1)       + mz * 2*(0.5 - lq1*lq1 - lq2*lq2);
    
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;


  // estimated direction of gravity and flux (v and w)
  vx = 2*(lq1*lq3 - lq0*lq2);
  vy = 2*(lq0*lq1 + lq2*lq3);
  vz = lq0*lq0 - lq1*lq1 - lq2*lq2 + lq3*lq3;
      
  wx = bx * 2*(0.5 - lq2*lq2 - lq3*lq3) + bz * 2*(lq1*lq3 - lq0*lq2);
  wy = bx * 2*(lq1*lq2 - lq0*lq3)       + bz * 2*(lq0*lq1 + lq2*lq3);
  //wz = bx * 2*(lq0*lq2 + lq1*lq3)       + bz * 2*(0.5 - lq1*lq1 - lq2*lq2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  exAcc = (vy*az - vz*ay);
  eyAcc = (vz*ax - vx*az);
  ezAcc = (vx*ay - vy*ax);
    
  ezMag = (mx*wy - my*wx);
    
  // integral error scaled integral gain
  lexInt = lexInt + exAcc*lkiAcc;
  leyInt = leyInt + eyAcc*lkiAcc;
  lezInt = lezInt + ezAcc*lkiAcc;

  // adjusted gyroscope measurements
  gx = gx + lkpAcc*exAcc + lexInt;
  gy = gy + lkpAcc*eyAcc + leyInt;
  gz = gz + lkpAcc*ezAcc + ezMag*lkpMag + lezInt;

  
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

void initializeBaseHeadingParam(float rollAngle, float pitchAngle, float yawAngle) {
  headingAngle[XAXIS] = rollAngle;
  headingAngle[YAXIS] = pitchAngle;
  headingAngle[ZAXIS] = yawAngle;
}



void localInitializeHeadingFusion(float ax, float ay, float az, float hdgX, float hdgY)  
{
  float norm = 1, rollAngle, pitchAngle, pi, tmp;
  float yawAngle = atan2(hdgY, hdgX);

  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  tmp = atan2(ay, sqrt(ax*ax+az*az));
  pi = radians(180);

  if (az < 0) { //board up
	tmp = -tmp;
  } else {		//board upside down
	if (ay >= 0) {
	  tmp -= pi;
	} else {
	  tmp += pi;
	}
  }
  
  rollAngle = tmp;
  pitchAngle = atan2(ax, sqrt(ay*ay+az*az));

  initializeBaseHeadingParam(rollAngle, pitchAngle, yawAngle);

  lq0 = cos(rollAngle/2)*cos(pitchAngle/2)*cos(yawAngle/2) + sin(rollAngle/2)*sin(pitchAngle/2)*sin(yawAngle/2);
  lq1 = sin(rollAngle/2)*cos(pitchAngle/2)*cos(yawAngle/2) - cos(rollAngle/2)*sin(pitchAngle/2)*sin(yawAngle/2);
  lq2 = cos(rollAngle/2)*sin(pitchAngle/2)*cos(yawAngle/2) + sin(rollAngle/2)*cos(pitchAngle/2)*sin(yawAngle/2);
  lq3 = cos(rollAngle/2)*cos(pitchAngle/2)*sin(yawAngle/2) - sin(rollAngle/2)*sin(pitchAngle/2)*cos(yawAngle/2);

  lexInt = 0.0;
  leyInt = 0.0;
  lezInt = 0.0;

  lkpAcc = 0.2;
  lkiAcc = 0.0005;
    
  lkpMag = 0.2;//2.0;
  lkiMag = 0.0005;//0.005;
}

void initializeHeadingFusion()
{
	localInitializeHeadingFusion(0.0, 0.0, -9.8, hdgX, hdgY);
}


////////////////////////////////////////////////////////////////////////////////
// Calculate ARG
////////////////////////////////////////////////////////////////////////////////
void localCalculateHeading(float rollRate,          float pitchRate,    float yawRate,  
                           float longitudinalAccel, float lateralAccel, float verticalAccel, 
                           float measuredMagX,      float measuredMagY, float measuredMagZ,
				           float G_Dt) {
    
  headingUpdate(rollRate,          pitchRate,    yawRate, 
             longitudinalAccel, lateralAccel, verticalAccel,  
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
                     filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], 
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