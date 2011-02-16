/*
  AeroQuad v2.2 - Feburary 2011
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

#include "FlightAngleDCM.h"

#include <AQMath.h>

FlightAngleDCM::FlightAngleDCM()
{
}

//**********************************************************************************************
//
//  Matrix Update
//
//**********************************************************************************************
void FlightAngleDCM::matrixUpdate(float G_Dt) 
{
  float gyroVector[3];
  gyroVector[0]=-(_gyro->getData(PITCH) * Gyro_Gain); //gyro y roll
  gyroVector[1]=_gyro->getData(ROLL) * Gyro_Gain; //gyro x pitch
  gyroVector[2]=_gyro->getData(YAW) * Gyro_Gain; //gyro Z yaw
  vectorAdd(3, &Omega[0], &gyroVector[0], &Omega_I[0]);   // adding integrator
  vectorAdd(3, &Omega_Vector[0], &Omega[0], &Omega_P[0]);  // adding proportional
    
  // Low pass filter on accelerometer data (to filter vibrations)
  Accel_Vector[0]=Accel_Vector[0]*0.6 + (float)-_accel->rateG(ROLL)*100.0; // acc x
  Accel_Vector[1]=Accel_Vector[1]*0.6 + (float)_accel->rateG(PITCH)*100.0; // acc y
  Accel_Vector[2]=Accel_Vector[2]*0.6 + (float)_accel->rateG(ZAXIS)*100.0; // acc z
    
  float updateMatrix[9];
  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt*Omega_Vector[2];  // -z
  updateMatrix[2] =  G_Dt*Omega_Vector[1];  //  y
  updateMatrix[3] =  G_Dt*Omega_Vector[2];  //  z
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt*Omega_Vector[0];  // -x
  updateMatrix[6] = -G_Dt*Omega_Vector[1];  // -y
  updateMatrix[7] =  G_Dt*Omega_Vector[0];  //  x
  updateMatrix[8] =  0;

  float temporaryMatrix[9];
  matrixMultiply(3, 3, 3, temporaryMatrix, DCM_Matrix, updateMatrix); //a*b=c
  matrixAdd(3, 3, DCM_Matrix, DCM_Matrix, temporaryMatrix);
}

//**********************************************************************************************
//
//  Normalize
//
//**********************************************************************************************
void FlightAngleDCM::normalize(void) 
{
  float temporary[9];
  float renorm=0;
   
  float error= -vectorDotProduct(3, &DCM_Matrix[0], &DCM_Matrix[3])*.5;         // eq.18

  vectorScale(3, &temporary[0], &DCM_Matrix[3], error);                   // eq.19
  vectorScale(3, &temporary[3], &DCM_Matrix[0], error);                   // eq.19
    
  vectorAdd(6, &temporary[0], &temporary[0], &DCM_Matrix[0]);             // eq.19
   
  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);          // c= a x b //eq.20
    
  for(byte v=0; v<9; v+=3) 
  {
    renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));   // eq.21
    vectorScale(3, &DCM_Matrix[v], &temporary[v], renorm);
  }
}

//**********************************************************************************************
//
//  Drift Correction
//
//**********************************************************************************************
void FlightAngleDCM::driftCorrection(void) 
{
  //Compensation the Roll, Pitch and Yaw drift. 
  //float        errorCourse;
  //static float Scaled_Omega_P[3];
  float scaledOmegaI[3];
  float errorRollPitch[3];
    
  //*****Roll and Pitch***************
 
  // Calculate the magnitude of the accelerometer vector
  // Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  // Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // Accel_weight = constrain(1 - 4*abs(1 - Accel_magnitude),0,1);
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  // Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
  
  vectorCrossProduct(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[6]); //adjust the ground of reference
  // Limit max errorRollPitch to limit max Omega_P and Omega_I
  #define MAX_ERROR 50
  errorRollPitch[0] = constrain(errorRollPitch[0],-MAX_ERROR,MAX_ERROR);
  errorRollPitch[1] = constrain(errorRollPitch[1],-MAX_ERROR,MAX_ERROR);
  errorRollPitch[2] = constrain(errorRollPitch[2],-MAX_ERROR,MAX_ERROR);
  vectorScale(3, &Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH);
    
  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], Ki_ROLLPITCH);
  vectorAdd(3, Omega_I, Omega_I, scaledOmegaI);
    
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading 
  /*if (MAGNETOMETER == 1) {
	  float errorYaw[3];

    errorCourse= (DCM_Matrix[0][0]*APM_Compass.Heading_Y) - (DCM_Matrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
    Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
   
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
    
    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
  }*/
}

//**********************************************************************************************
//
//  Euler Angles
//
//**********************************************************************************************
void FlightAngleDCM::eulerAngles(void)
{
  _angle[ROLL] =  degrees(asin(-DCM_Matrix[6]));
  _angle[PITCH] = degrees(atan2(DCM_Matrix[7],DCM_Matrix[8]));
  _angle[YAW] =   degrees(atan2(DCM_Matrix[3],DCM_Matrix[0]));
} 
  

  
void FlightAngleDCM::initialize(void) 
{
  for (byte i=0; i<3; i++) 
  {
    Accel_Vector[i]            = 0;  // Store the acceleration in a vector
    Omega_Vector[i]            = 0;  // Corrected Gyro_Vector data
    Omega_P[i]                 = 0;  // Omega Proportional correction
    Omega_I[i]                 = 0;  // Omega Integrator
    Omega[i]                   = 0;
  }
  DCM_Matrix[0]       = 1;
  DCM_Matrix[1]       = 0;
  DCM_Matrix[2]       = 0;
  DCM_Matrix[3]       = 0;
  DCM_Matrix[4]       = 1;
  DCM_Matrix[5]       = 0;
  DCM_Matrix[6]       = 0;
  DCM_Matrix[7]       = 0;
  DCM_Matrix[8]       = 1;

  errorCourse = 0;
  COGX = 0; //Course overground X axis
  COGY = 1; //Course overground Y axis    
  dt = 0;
  Gyro_Gain = radians(_gyro->getScaleFactor());
  _type = DCM;
  Kp_ROLLPITCH = 0.0014;
  Ki_ROLLPITCH = 0.00000012; // was 0.00000015
}
  
void FlightAngleDCM::calculate(float G_Dt) 
{
  matrixUpdate(G_Dt); 
  normalize();
  driftCorrection();
  eulerAngles();
}
  
float FlightAngleDCM::getGyroUnbias(byte axis) 
{ 
  if (axis == ROLL)
  {
    return degrees(Omega[1]);
  }
  else if (axis == PITCH)
  {
    return degrees(-Omega[0]);
  }
  else
  {
    return degrees(Omega[2]);
  }
}

