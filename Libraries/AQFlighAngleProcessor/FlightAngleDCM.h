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

#ifndef _AQ_FLIGHT_ANGLE_PROCESSOR_DCM_FILTER_H_
#define _AQ_FLIGHT_ANGLE_PROCESSOR_DCM_FILTER_H_

#include "FlightAngleProcessor.h"

class FlightAngleDCM : public FlightAngleProcessor 
{
private:
  float dt;
  float Gyro_Gain;  // jihlein; Replaced X, Y, and Z gyro gains with single gain
  float DCM_Matrix[9];
  float Accel_Vector[3];
  float Omega_Vector[3];
  float Omega_P[3];
  float Omega_I[3];
  float Omega[3];
  float errorCourse;
  float COGX; //Course overground X axis
  float COGY; //Course overground Y axis
  float Kp_ROLLPITCH;
  float Ki_ROLLPITCH;


  //**********************************************************************************************
  //
  //  Matrix Update
  //
  //**********************************************************************************************
  void matrixUpdate(float G_Dt);

  //**********************************************************************************************
  //
  //  Normalize
  //
  //**********************************************************************************************
  void normalize(void);

  //**********************************************************************************************
  //
  //  Drift Correction
  //
  //**********************************************************************************************
  void driftCorrection(void);

  //**********************************************************************************************
  //
  //  Euler Angles
  //
  //**********************************************************************************************
  void eulerAngles(void);

public:  
  FlightAngleDCM();
  
  void initialize(void);
  
  void calculate(float G_Dt);
  
  float getGyroUnbias(byte axis);

  void calibrate(void) {}
};

#endif