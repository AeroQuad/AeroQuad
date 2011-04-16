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

#ifndef _AEROQUAD_KINEMATICS_DCM_H_
#define _AEROQUAD_KINEMATICS_DCM_H_

#include <WProgram.h>
#include "Kinematics.h"

class Kinematics_DCM : public Kinematics {
private:
  float dcmMatrix[9];
  float omegaP[3];
  float omegaI[3];
  float omega[3];
  float errorCourse;
  float kpRollPitch;
  float kiRollPitch;
  float kpYaw;
  float kiYaw;

  ////////////////////////////////////////////////////////////////////////////////
  // Matrix Update
  ////////////////////////////////////////////////////////////////////////////////
  void matrixUpdate(float p, float q, float r, unsigned long G_Dt);

  ////////////////////////////////////////////////////////////////////////////////
  // Normalize
  ////////////////////////////////////////////////////////////////////////////////
  void normalize(void);

  ////////////////////////////////////////////////////////////////////////////////
  // Drift Correction
  ////////////////////////////////////////////////////////////////////////////////
  void driftCorrection(float ax, float ay, float az, float oneG, float magX, float magY);

  
  ////////////////////////////////////////////////////////////////////////////////
  // Euler Angles
  ////////////////////////////////////////////////////////////////////////////////
  void eulerAngles(void);
  
  ////////////////////////////////////////////////////////////////////////////////
  // Earth Axis Accels
  ////////////////////////////////////////////////////////////////////////////////
  void earthAxisAccels(float ax, float ay, float az, float oneG);
  
public:
  Kinematics_DCM() {}
  
  ////////////////////////////////////////////////////////////////////////////////
  // Initialize DCM
  ////////////////////////////////////////////////////////////////////////////////
  void initialize(float hdgX, float hdgY);
  
  ////////////////////////////////////////////////////////////////////////////////
  // Calculate DCM
  ////////////////////////////////////////////////////////////////////////////////
  void calculate(float rollRate,            float pitchRate,      float yawRate,  \
                 float longitudinalAccel,   float lateralAccel,   float verticalAccel, \
                 float oneG,                float magX,           float magY,
				 unsigned long G_Dt);
  
  float getGyroUnbias(byte axis);
};

#endif