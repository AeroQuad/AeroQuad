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

#ifndef _AEROQUAD_KINEMATICS_H_
#define _AEROQUAD_KINEMATICS_H_

#include <WProgram.h>

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

class Kinematics {
public:
  byte type;
  float angle[3];
  float gyroAngle[2];
  float correctedRateVector[3];
  float earthAccel[3];
  
  Kinematics(void);
  
  virtual void initialize(float hdgX, float hdgY) {}
  virtual void calculate(float rollRate,           float pitchRate,     float yawRate,       \
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel, \
                         float oneG,               float magX,          float magY,
						 unsigned long G_Dt) {}
						 
  virtual float getGyroUnbias(byte axis) {}
 
  // returns the angle of a specific axis in SI units (radians)
  const float getData(byte axis);
  
  // return heading as +PI/-PI
  const float getHeading(byte axis);
  
  // This really needs to be in Radians to be consistent
  // I'll fix later - AKA
  // returns heading in degrees as 0-360
  const float getDegreesHeading(byte axis);
  
  const byte getType(void);
};

#endif


