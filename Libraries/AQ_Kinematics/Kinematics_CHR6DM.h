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

#ifndef _AQ_KINEMATICS_CHR6DM_
#define _AQ_KINEMATICS_CHR6DM_

#include "Kinematics.h"
#include <Platform_CHR6DM.h>
#include <Gyroscope_CHR6DM.h>

float zeroRoll       = 0.0;
float zeroPitch 	 = 0.0;
float CHR_RollAngle  = 0.0;
float CHR_PitchAngle = 0.0;

CHR6DM *kinematicsChr6dm;

void initializeKinematics(float hdgX, float hdgY) {
  initializeBaseKinematicsParam(hdgX,hdgY);
  calibrateKinematics();
}

void calculateKinematics(float rollRate,           float pitchRate,     float yawRate,       
				         float longitudinalAccel,  float lateralAccel,  float verticalAccel, 
				         float oneG,               float magX,          float magY,
				         float G_Dt) {
				 
  kinematicsAngle[XAXIS]  =  kinematicsChr6dm->data.roll - zeroRoll;
  kinematicsAngle[YAXIS] =  kinematicsChr6dm->data.pitch - zeroPitch;
  CHR_RollAngle = kinematicsAngle[XAXIS]; //ugly since gotta access through accel class
  CHR_PitchAngle = kinematicsAngle[YAXIS];
}
  
 void calibrateKinematics() {
  zeroRoll = kinematicsChr6dm->data.roll;
  zeroPitch = kinematicsChr6dm->data.pitch;
}
  
float getGyroUnbias(byte axis) {
  return gyroRate[axis];
}


#endif
