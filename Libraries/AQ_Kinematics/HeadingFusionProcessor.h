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

/*
 * eventually, it's about the normal DCM processor and I KNOW that this is heavy!
 * Still, this is the best result I did get with my knowledge that give good attitude
 * estimator with the AGR and pretty good true heading computation at the same time
 * 
 * @Kenny9999
 * I'm open to anything more lighweight working and FLIGHT TESTED
 */
#ifndef _AQ_HEADING_FUSION_PROCESSOR_DCM_
#define _AQ_HEADING_FUSION_PROCESSOR_DCM_

#if defined UseGPS
  #include "MagnetometerDeclinationDB.h"
#endif  

float dcmMatrix[9] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float omegaP[3] = {0.0,0.0,0.0};
float omegaI[3] = {0.0,0.0,0.0};
float omega[3] = {0.0,0.0,0.0};
float kpRollPitch = 0.0;
float kiRollPitch = 0.0;
float kpYaw = 0.0;
float kiYaw = 0.0;
float accelMagnitude = 0.0;
float accelWeight = 0.0;

float trueNorthHeading = 0.0;

float compassDeclination = 0.0;

////////////////////////////////////////////////////////////////////////////////
// Matrix Update
////////////////////////////////////////////////////////////////////////////////
void matrixUpdate(float p, float q, float r, float G_Dt) 
{
  float rateGyroVector[3];
  float temporaryMatrix[9];
  float updateMatrix[9];
  
  rateGyroVector[XAXIS] = p;
  rateGyroVector[YAXIS] = q;
  rateGyroVector[ZAXIS] = r;
  
  vectorSubtract(3, &omega[XAXIS], &rateGyroVector[XAXIS], &omegaI[XAXIS]);
  vectorSubtract(3, &correctedRateVector[XAXIS], &omega[XAXIS], &omegaP[XAXIS]); 
  
  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt * correctedRateVector[ZAXIS];  // -r
  updateMatrix[2] =  G_Dt * correctedRateVector[YAXIS];  //  q
  updateMatrix[3] =  G_Dt * correctedRateVector[ZAXIS];  //  r
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt * correctedRateVector[XAXIS];  // -p
  updateMatrix[6] = -G_Dt * correctedRateVector[YAXIS];  // -q
  updateMatrix[7] =  G_Dt * correctedRateVector[XAXIS];  //  p
  updateMatrix[8] =  0; 

  matrixMultiply(3, 3, 3, temporaryMatrix, dcmMatrix, updateMatrix); 
  matrixAdd(3, 3, dcmMatrix, dcmMatrix, temporaryMatrix);
}

////////////////////////////////////////////////////////////////////////////////
// Normalize
////////////////////////////////////////////////////////////////////////////////
void normalize() 
{
  float temporary[9];
 
  float error = -vectorDotProduct(3, &dcmMatrix[0], &dcmMatrix[3]) * 0.5;  // eq.18

  vectorScale(3, &temporary[0], &dcmMatrix[3], error);                     // eq.19
  vectorScale(3, &temporary[3], &dcmMatrix[0], error);                     // eq.19
  
  vectorAdd(6, &temporary[0], &temporary[0], &dcmMatrix[0]);               // eq.19
  
  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);           // eq.20
  
  for(byte v = 0; v < 9; v+=3) {
    float renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));  // eq.21
    vectorScale(3, &dcmMatrix[v], &temporary[v], renorm);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Drift Correction
////////////////////////////////////////////////////////////////////////////////
void driftCorrection(float ax, float ay, float az, float oneG, float magX, float magY) 
{
  //  Compensation of the Roll, Pitch and Yaw drift. 
  float errorRollPitch[3];
  float errorYaw[3];
  float scaledOmegaP[3];
  float scaledOmegaI[3];
  
  //  Roll and Pitch Compensation
  float accelVector[3];
  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;

  if (accelMagnitude == 0.0) {
    // Calculate the magnitude of the accelerometer vector
    accelMagnitude = (sqrt(accelVector[XAXIS] * accelVector[XAXIS] + 
                         accelVector[YAXIS] * accelVector[YAXIS] + 
                         accelVector[ZAXIS] * accelVector[ZAXIS])) / oneG;
                         
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    accelWeight = constrain(1 - 2 * fabs(1 - accelMagnitude), 0, 1);
  }
  
  vectorCrossProduct(&errorRollPitch[0], &accelVector[0], &dcmMatrix[6]);
  vectorScale(3, &omegaP[0], &errorRollPitch[0], kpRollPitch * accelWeight);
  
  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], kiRollPitch * accelWeight);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);

  //  Yaw Compensation
  float errorCourse = (dcmMatrix[0] * magY) - (dcmMatrix[3] * magX);
  vectorScale(3, errorYaw, &dcmMatrix[6], errorCourse);
 
  vectorScale(3, &scaledOmegaP[0], &errorYaw[0], kpYaw);
  vectorAdd(3, omegaP, omegaP, scaledOmegaP);
  
  vectorScale(3, &scaledOmegaI[0] ,&errorYaw[0], kiYaw);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);
}



////////////////////////////////////////////////////////////////////////////////
// Initialize Heading Fusion
////////////////////////////////////////////////////////////////////////////////
void initializeHeadingFusion(float hdgX, float hdgY) 
{
  for (byte i=0; i<3; i++) {
    omegaP[i] = 0;
    omegaI[i] = 0;
  }
  dcmMatrix[0] =  hdgX;
  dcmMatrix[1] = -hdgY;
  dcmMatrix[2] =  0;
  dcmMatrix[3] =  hdgY;
  dcmMatrix[4] =  hdgX;
  dcmMatrix[5] =  0;
  dcmMatrix[6] =  0;
  dcmMatrix[7] =  0;
  dcmMatrix[8] =  1;

  kpRollPitch = 0.05;       // alternate 0.1;
  kiRollPitch = 0.0001;     // alternate 0.0002;
    
  kpYaw = -0.05;            // alternate -0.05;
  kiYaw = -0.0001;          // alternate -0.0001;
    
}
  
////////////////////////////////////////////////////////////////////////////////
// Initialize Heading Fusion
////////////////////////////////////////////////////////////////////////////////
void calculateHeading(float rollRate,            float pitchRate,      float yawRate,  
                      float longitudinalAccel,   float lateralAccel,   float verticalAccel, 
                      float oneG,                float magX,           float magY,
				      float G_Dt) {
    
  matrixUpdate(rollRate, pitchRate, yawRate, G_Dt); 
  normalize();
  driftCorrection(longitudinalAccel, lateralAccel, verticalAccel, oneG, magX, magY);
  
  trueNorthHeading =  atan2(dcmMatrix[3], dcmMatrix[0]);
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


#if defined UseGPS
  void setDeclinationLocation(long lat, long lon) {
    // get declination ( in radians )
    compassDeclination = getMagnetometerDeclination(lat, lon);    
  }
#endif  


#endif

