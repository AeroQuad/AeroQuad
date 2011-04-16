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

#include "Kinematics_DCM.h"

#include <Axis.h>
#include <AQMath.h>

void Kinematics_DCM::matrixUpdate(float p, float q, float r, unsigned long G_Dt) 
{
  float rateGyroVector[3];
  float temporaryMatrix[9];
  float updateMatrix[9];
  
  rateGyroVector[ROLL]  = p;
  rateGyroVector[PITCH] = q;
  rateGyroVector[YAW]   = r;
  
  vectorSubtract(3, &omega[ROLL], &rateGyroVector[ROLL], &omegaI[ROLL]);
  vectorSubtract(3, &correctedRateVector[ROLL], &omega[ROLL], &omegaP[ROLL]); 
  
  //Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter
  
  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt * correctedRateVector[YAW];    // -r
  updateMatrix[2] =  G_Dt * correctedRateVector[PITCH];  //  q
  updateMatrix[3] =  G_Dt * correctedRateVector[YAW];    //  r
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt * correctedRateVector[ROLL];   // -p
  updateMatrix[6] = -G_Dt * correctedRateVector[PITCH];  // -q
  updateMatrix[7] =  G_Dt * correctedRateVector[ROLL];   //  p
  updateMatrix[8] =  0; 

  matrixMultiply(3, 3, 3, temporaryMatrix, dcmMatrix, updateMatrix); 
  matrixAdd(3, 3, dcmMatrix, dcmMatrix, temporaryMatrix);
}

void Kinematics_DCM::normalize(void) 
{
  float error=0;
  float temporary[9];
  float renorm=0;
  
  error= -vectorDotProduct(3, &dcmMatrix[0], &dcmMatrix[3]) * 0.5;         // eq.18

  vectorScale(3, &temporary[0], &dcmMatrix[3], error);                     // eq.19
  vectorScale(3, &temporary[3], &dcmMatrix[0], error);                     // eq.19
  
  vectorAdd(6, &temporary[0], &temporary[0], &dcmMatrix[0]);               // eq.19
  
  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);           // eq.20
  
  for(byte v=0; v<9; v+=3) {
    renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));  // eq.21
    vectorScale(3, &dcmMatrix[v], &temporary[v], renorm);
  }
}

void Kinematics_DCM::driftCorrection(float ax, float ay, float az, float oneG, float magX, float magY) 
{
  //  Compensation of the Roll, Pitch and Yaw drift. 
  float accelMagnitude;
  float accelVector[3];
  float accelWeight;
  float errorRollPitch[3];
#ifdef HeadingMagHold  
  float errorCourse;
  float errorYaw[3];
  float scaledOmegaP[3];
#endif  
  float scaledOmegaI[3];
  
  //  Roll and Pitch Compensation
  
  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;

  // Calculate the magnitude of the accelerometer vector
  accelMagnitude = (sqrt(accelVector[XAXIS] * accelVector[XAXIS] + \
                         accelVector[YAXIS] * accelVector[YAXIS] + \
                         accelVector[ZAXIS] * accelVector[ZAXIS])) / oneG;
                         
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // accelWeight = constrain(1 - 4*abs(1 - accelMagnitude),0,1);
  
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  accelWeight = constrain(1 - 2 * abs(1 - accelMagnitude), 0, 1);
  
  vectorCrossProduct(&errorRollPitch[0], &accelVector[0], &dcmMatrix[6]);
  vectorScale(3, &omegaP[0], &errorRollPitch[0], kpRollPitch * accelWeight);
  
  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], kiRollPitch * accelWeight);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);
  
  //  Yaw Compensation
  
#ifdef HeadingMagHold
  errorCourse = (dcmMatrix[0] * magY) - (dcmMatrix[3] * magX);
  vectorScale(3, errorYaw, &dcmMatrix[6], errorCourse);

  vectorScale(3, &scaledOmegaP[0], &errorYaw[0], kpYaw);
  vectorAdd(3, omegaP, omegaP, scaledOmegaP);
  
  vectorScale(3, &scaledOmegaI[0] ,&errorYaw[0], kiYaw);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);
#else
  omegaP[YAW] = 0.0;
  omegaI[YAW] = 0.0;
#endif
}

void Kinematics_DCM::eulerAngles(void)
{
  angle[ROLL]  =  atan2(dcmMatrix[7], dcmMatrix[8]);
  angle[PITCH] =  -asin(dcmMatrix[6]);
  angle[YAW]   =  atan2(dcmMatrix[3], dcmMatrix[0]);
} 
  
void Kinematics_DCM::earthAxisAccels(float ax, float ay, float az, float oneG)
{
  float accelVector[3];
  
  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;
  
  earthAccel[XAXIS] = vectorDotProduct(3, &dcmMatrix[0], &accelVector[0]);
  earthAccel[YAXIS] = vectorDotProduct(3, &dcmMatrix[3], &accelVector[0]);
  earthAccel[ZAXIS] = vectorDotProduct(3, &dcmMatrix[6], &accelVector[0]) + oneG;
} 
  
void Kinematics_DCM::initialize(float hdgX, float hdgY) 
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

  // Original from John
//    kpRollPitch = 1.6;
//    kiRollPitch = 0.005;
    
//    kpYaw = -1.6;
//    kiYaw = -0.005;
/*    
  // released in 2.2
  kpRollPitch = 1.0;
  kiRollPitch = 0.002;
    
  kpYaw = -1.0;
  kiYaw = -0.002;
*/
  kpRollPitch = 0.1;        // alternate 0.05;
  kiRollPitch = 0.0002;     // alternate 0.0001;
  
  kpYaw = -0.1;             // alternate -0.05;
  kiYaw = -0.0002;          // alternate -0.0001;
}
  
void Kinematics_DCM::calculate(float rollRate,            float pitchRate,      float yawRate,  \
							   float longitudinalAccel,   float lateralAccel,   float verticalAccel, \
                               float oneG,                float magX,           float magY,
			                   unsigned long G_Dt) {
    
  matrixUpdate(rollRate, pitchRate, yawRate, G_Dt); 
  normalize();
  driftCorrection(longitudinalAccel, lateralAccel, verticalAccel, oneG, magX, magY);
  eulerAngles();
  earthAxisAccels(longitudinalAccel, lateralAccel, verticalAccel, oneG);
}
  
float Kinematics_DCM::getGyroUnbias(byte axis) {
  return correctedRateVector[axis];
}
  
  
