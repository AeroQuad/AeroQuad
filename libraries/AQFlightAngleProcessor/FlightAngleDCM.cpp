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

FlightAngleDCM::FlightAngleDCM(Gyroscope *gyro,Accelerometer *accel) : FlightAngleProcessor(gyro,accel)
{
//  _gyro = gyro;
//  _accel = accel;
}

void FlightAngleDCM::matrixUpdate(unsigned long G_Dt) 
{
  float gyroVector[3];
  gyroVector[0]=-(_gyro->getData(PITCH) * _gyroGain); //gyro y roll
  gyroVector[1]=_gyro->getData(ROLL) * _gyroGain; //gyro x pitch
  gyroVector[2]=_gyro->getData(YAW) * _gyroGain; //gyro Z yaw
  vectorAdd(3, &_omega[0], &gyroVector[0], &_omegaI[0]);   // adding integrator
  vectorAdd(3, &_omegaVector[0], &_omega[0], &_omegaP[0]);  // adding proportional
    
  // Low pass filter on accelerometer data (to filter vibrations)
  _accelVector[0]=_accelVector[0]*0.6 + (float)-_accel->rateG(ROLL)*100.0; // acc x
  _accelVector[1]=_accelVector[1]*0.6 + (float)_accel->rateG(PITCH)*100.0; // acc y
  _accelVector[2]=_accelVector[2]*0.6 + (float)_accel->rateG(ZAXIS)*100.0; // acc z
    
  float updateMatrix[9];
  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt*_omegaVector[2];  // -z
  updateMatrix[2] =  G_Dt*_omegaVector[1];  //  y
  updateMatrix[3] =  G_Dt*_omegaVector[2];  //  z
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt*_omegaVector[0];  // -x
  updateMatrix[6] = -G_Dt*_omegaVector[1];  // -y
  updateMatrix[7] =  G_Dt*_omegaVector[0];  //  x
  updateMatrix[8] =  0;

  float temporaryMatrix[9];
  matrixMultiply(3, 3, 3, temporaryMatrix, _dcmMatrix, updateMatrix); //a*b=c
  matrixAdd(3, 3, _dcmMatrix, _dcmMatrix, temporaryMatrix);
}

void FlightAngleDCM::normalize() 
{
  float temporary[9];
  float renorm=0;
  
  float error= -vectorDotProduct(3, &_dcmMatrix[0], &_dcmMatrix[3])*.5;         // eq.18
 
  vectorScale(3, &temporary[0], &_dcmMatrix[3], error);                   // eq.19
  vectorScale(3, &temporary[3], &_dcmMatrix[0], error);                   // eq.19
    
  vectorAdd(6, &temporary[0], &temporary[0], &_dcmMatrix[0]);             // eq.19
    
  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);          // c= a x b //eq.20
    
  for(byte v=0; v<9; v+=3) 
  {
    renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));   // eq.21
    vectorScale(3, &_dcmMatrix[v], &temporary[v], renorm);
  }
}

void FlightAngleDCM::driftCorrection() 
{
  //Compensation the Roll, Pitch and Yaw drift. 
  //float        _errorCourse;
  //static float Scaled_Omega_P[3];
  float scaledOmegaI[3];
  float errorRollPitch[3];
    
  //*****Roll and Pitch***************
  
  // Calculate the magnitude of the accelerometer vector
  // Accel_magnitude = sqrt(_accelVector[0]*_accelVector[0] + _accelVector[1]*_accelVector[1] + _accelVector[2]*_accelVector[2]);
  // Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // Accel_weight = constrain(1 - 4*abs(1 - Accel_magnitude),0,1);
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  // Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
  
  vectorCrossProduct(&errorRollPitch[0], &_accelVector[0], &_dcmMatrix[6]); //adjust the ground of reference
  // Limit max errorRollPitch to limit max _omegaP and _omegaI
  #define MAX_ERROR 50
  errorRollPitch[0] = constrain(errorRollPitch[0],-MAX_ERROR,MAX_ERROR);
  errorRollPitch[1] = constrain(errorRollPitch[1],-MAX_ERROR,MAX_ERROR);
  errorRollPitch[2] = constrain(errorRollPitch[2],-MAX_ERROR,MAX_ERROR);
  vectorScale(3, &_omegaP[0], &errorRollPitch[0], Kp_ROLLPITCH);
    
  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], Ki_ROLLPITCH);
  vectorAdd(3, _omegaI, _omegaI, scaledOmegaI);
    
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading 
  /*if (MAGNETOMETER == 1) {
    float errorYaw[3];
  
    _errorCourse= (_dcmMatrix[0][0]*APM_Compass.Heading_Y) - (_dcmMatrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
    Vector_Scale(errorYaw,&_dcmMatrix[2][0],_errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    
    Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
    Vector_Add(_omegaP,_omegaP,Scaled_Omega_P);//Adding  Proportional.
    
    Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
    Vector_Add(_omegaI,_omegaI,Scaled_Omega_I);//adding integrator to the _omegaI
  }*/
}

void FlightAngleDCM::eulerAngles()
{
  _angle[ROLL] =  degrees(asin(-_dcmMatrix[6]));
  _angle[PITCH] = degrees(atan2(_dcmMatrix[7],_dcmMatrix[8]));
  _angle[YAW] =   degrees(atan2(_dcmMatrix[3],_dcmMatrix[0]));
} 
  

  
  
void FlightAngleDCM::initialize() 
{
  for (byte i=0; i<3; i++) 
  {
    _accelVector[i]            = 0;  // Store the acceleration in a vector
    _omegaVector[i]            = 0;  // Corrected Gyro_Vector data
    _omegaP[i]                 = 0;  // Omega Proportional correction
    _omegaI[i]                 = 0;  // Omega Integrator
    _omega[i]                   = 0;
  }
  _dcmMatrix[0]       = 1;
  _dcmMatrix[1]       = 0;
  _dcmMatrix[2]       = 0;
  _dcmMatrix[3]       = 0;
  _dcmMatrix[4]       = 1;
  _dcmMatrix[5]       = 0;
  _dcmMatrix[6]       = 0;
  _dcmMatrix[7]       = 0;
  _dcmMatrix[8]       = 1;

  _errorCourse = 0;
  COGX = 0; //Course overground X axis
  COGY = 1; //Course overground Y axis    
  _gyroGain = radians(_gyro->getScaleFactor());
  _type = DCM;
  Kp_ROLLPITCH = 0.0014;
  Ki_ROLLPITCH = 0.00000012; // was 0.00000015
}
  
void FlightAngleDCM::calculate(unsigned long G_Dt) 
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
    return degrees(_omega[1]);
  }
  else if (axis == PITCH)
  {
    return degrees(-_omega[0]);
  }
  else
  {
    return degrees(_omega[2]);
  }
}
