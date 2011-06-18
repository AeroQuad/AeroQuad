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
#include <Gyroscope.h>

class Kinematics_CHR6DM : public Kinematics {
private:
  float zeroRoll;
  float zeroPitch;
  CHR6DM *chr6dm;
  Gyroscope* gyroscope;

public:
  Kinematics_CHR6DM() : Kinematics() {}
  
  void setChr6dm(CHR6DM *chr6dm) {
    this->chr6dm = chr6dm;
  }
  
  void setGyroscope(Gyroscope *gyroscope) {
    this->gyroscope = gyroscope;
  }

  void initialize(float hdgX, float hdgY) {
    calibrate();
  }

  void calculate(float rollRate,           float pitchRate,     float yawRate,       
				 float longitudinalAccel,  float lateralAccel,  float verticalAccel, 
				 float oneG,               float magX,          float magY,
				 float G_Dt) {
				 
    angle[ROLL]  =  chr6dm->data.roll - zeroRoll;
    angle[PITCH] =  chr6dm->data.pitch - zeroPitch;
    CHR_RollAngle = angle[ROLL]; //ugly since gotta access through accel class
    CHR_PitchAngle = angle[PITCH];
  }
  
   void calibrate() {
    zeroRoll = chr6dm->data.roll;
    zeroPitch = chr6dm->data.pitch;
  }
  
  float getGyroUnbias(byte axis) {
    return gyroscope->getRadPerSec(axis);
  }

};


#endif
