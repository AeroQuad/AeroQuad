/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AQ_HEADING_FUSION_PROCESSOR
#define _AQ_HEADING_FUSION_PROCESSOR


//float kpAcc = 0.0;                					// proportional gain governs rate of convergence to accelerometer
//float kiAcc = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
//float kpMag = 0.0;                					// proportional gain governs rate of convergence to magnetometer
//float kiMag = 0.0;                					// integral gain governs rate of convergence of gyroscope biases
float lhalfT = 0.0;                					// half the sample period
float lq0 = 0.0, lq1 = 0.0, lq2 = 0.0, lq3 = 0.0;       // quaternion elements representing the estimated orientation
float lexInt = 0.0, leyInt = 0.0, lezInt = 0.0;  		// scaled integral error

float trueNorthHeading = 0.0;


void headingUpdate(float mx, float my, float mz, float G_Dt) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float q0i, q1i, q2i, q3i;
  float exAcc, eyAcc, ezAcc;
  float exMag, eyMag, ezMag;
    
  lhalfT = G_Dt/2;
  
  // auxiliary variables to reduce number of repeated operations
  float q0q0 = lq0*lq0;
  float q0q1 = lq0*lq1;
  float q0q2 = lq0*lq2;
  float q0q3 = lq0*lq3;
  float q1q1 = lq1*lq1;
  float q1q2 = lq1*lq2;
  float q1q3 = lq1*lq3;
  float q2q2 = lq2*lq2;   
  float q2q3 = lq2*lq3;
  float q3q3 = lq3*lq3;          
    	
  // normalise the measurements
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
//  lexInt = lexInt + exAcc*kiAcc + exMag*kiMag;
//  leyInt = leyInt + eyAcc*kiAcc + eyMag*kiMag;
//  lezInt = lezInt + ezAcc*kiAcc + ezMag*kiMag;
    	
  // adjusted gyroscope measurements
//  float l_gx = gx + exAcc*kpAcc + exMag*kpMag + lexInt;
//  float l_gy = gy + eyAcc*kpAcc + eyMag*kpMag + leyInt;
//  float l_gz = gz + ezAcc*kpAcc + ezMag*kpMag + lezInt;

  // adjusted gyroscope measurements
  float l_gx = gx + lexInt;
  float l_gy = gy + leyInt;
  float l_gz = gz + lezInt;
  
  // integrate quaternion rate and normalise
  q0i = (-lq1*l_gx - lq2*l_gy - lq3*l_gz) * lhalfT;
  q1i = ( lq0*l_gx + lq2*l_gz - lq3*l_gy) * lhalfT;
  q2i = ( lq0*l_gy - lq1*l_gz + lq3*l_gx) * lhalfT;
  q3i = ( lq0*l_gz + lq1*l_gy - lq2*l_gx) * lhalfT;
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
  trueNorthHeading = atan2(2 * (lq0*lq3 + lq1*lq2), 1 - 2 *(lq2*lq2 + lq3*lq3));
}

  
////////////////////////////////////////////////////////////////////////////////
// Initialize MARG
////////////////////////////////////////////////////////////////////////////////
void initializeHeadingFusion(float hdgX, float hdgY) 
{
  float hdg = atan2(hdgY, hdgX);
    
  lq0 = cos(hdg/2);
  lq1 = 0;
  lq2 = 0;
  lq3 = sin(hdg/2);
  lexInt = 0.0;
  leyInt = 0.0;
  lezInt = 0.0;

/*  kpAcc = 0.0;//0.2;
  kiAcc = 0.0;//0.0005;
    
  kpMag = 0.0;//0.2;//2.0;
  kiMag = 0.0;//0.0005;//0.005;
*/
}
  
////////////////////////////////////////////////////////////////////////////////
// Calculate MARG
////////////////////////////////////////////////////////////////////////////////

void calculateHeading(float measuredMagX, float measuredMagY, float measuredMagZ, float G_Dt) {
				 
  headingUpdate(measuredMagX, measuredMagY, measuredMagZ, G_Dt);
  headingEulerAngles();
}

#endif

