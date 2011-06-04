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

#ifndef _AQ_KINEMATICS_
#define _AQ_KINEMATICS_

#include <Axis.h>

#define CF 0
#define KF 1
#define DCM 2
#define ARG 3
#define MARG 4

// This class is responsible for calculating vehicle attitude
class Kinematics {
protected:
  byte type;
  float angle[3];
  float gyroAngle[2];
  float correctedRateVector[3];
  float earthAccel[3];

public:  
  Kinematics(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++)
      angle[axis] = 0.0;
    gyroAngle[ROLL] = 0;
    gyroAngle[PITCH] = 0;
  }
  
  virtual void initialize(float hdgX, float hdgY);
  virtual void calculate(float rollRate,           float pitchRate,     float yawRate,       
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel, 
                         float oneG,               float magX,          float magY,
						 float G_Dt);
  virtual float getGyroUnbias(byte axis);
  virtual void calibrate();
 
  // returns the angle of a specific axis in SI units (radians)
  const float getData(byte axis) {
    return angle[axis];
  }
  // return heading as +PI/-PI
  const float getHeading(byte axis) {
    return(angle[axis]);
  }
  
  // This really needs to be in Radians to be consistent
  // I'll fix later - AKA
  // returns heading in degrees as 0-360
  const float getDegreesHeading(byte axis) {
    float tDegrees;
    
    tDegrees = degrees(angle[axis]);
    if (tDegrees < 0.0)
      return (tDegrees + 360.0);
    else
      return (tDegrees);
  }
  
  const byte getType(void) {
    // This is set in each subclass to identify which algorithm used
    return type;
  }
};

#endif

