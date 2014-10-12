/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#include "GlobalDefined.h"

double halfT = 0.0;                		
double q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
double previousEx = 0.0;
double previousEy = 0.0;
double previousEz = 0.0;


// This class is responsible for calculating vehicle attitude
byte kinematicsType = 0;
double kinematicsAngle[3] = {0.0,0.0,0.0};
double correctedRateVector[3] = {0.0,0.0,0.0};
double earthAccel[3] = {0.0,0.0,0.0};



#define HardFilter(O,N)  ((O)*0.9f+(N)*0.1f)

#define DEFAULT_Kp 0.4  // 0.2
#define DEFAULT_Ki 0.0005 // 0.0005

double accConfidence      = 1.0f;
double accConfidenceDecay = 1.0f / sqrt(0.75f);	// @todo, accelCutOff should go into eeprom... if it work


void calculateAccConfidence(double accMag)
{
	// G.K. Egan (C) computes confidence in accelerometers when
	// aircraft is being accelerated over and above that due to gravity
	static double accMagP = 1.0f;
	accMag /= accelOneG;  // HJI Added to convert MPS^2 to G's
	accMag  = HardFilter(accMagP, accMag );
	accMagP = accMag;
	accConfidence = constrain(1.0 - (accConfidenceDecay * sqrt(fabs(accMag - 1.0f))), 0.01f, 1.0f);
}


void initializeBaseKinematicParam() {

  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    kinematicsAngle[axis] = 0.0;
  }
}

//void initializeKinematics(double hdgX, double hdgY);
	

const double kinematicsGetDegreesHeading(byte axis) {
    
  double tDegrees = degrees(kinematicsAngle[axis]);
  if (tDegrees < 0.0) {
    return (tDegrees + 360.0);
  }
  else {
    return (tDegrees);
  }
}

#endif

