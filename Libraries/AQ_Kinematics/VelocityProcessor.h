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

#ifndef _AQ_VELOCITY_PROCESSOR_
#define _AQ_VELOCITY_PROCESSOR_

#include "GlobalDefined.h"
#include "FifteenFloatBuffer.h"

float timeConstantZ = 5.0;    
float k1_z = 3 / timeConstantZ;
float k2_z = 10 / (timeConstantZ*timeConstantZ);
float k3_z = 1 / (timeConstantZ*timeConstantZ*timeConstantZ);

float computedZVelocity = 0.0;
float currentComputedZVelocity = 0.0;
float zErrorPosition = 0.0;
float zPositionCorrection = 0.0;
float accelZCorrection = 0.0;
float baseZPosition = 0.0;
FloatBuffer15Size zBasePositionHistoryBuffer;

void computeVelocity(float filteredAccelZ, float dt);
void computeVelocityErrorFromBaroAltitude(float baroAltitude);


void computeVelocityErrorFromBaroAltitude(float baroAltitude)
{
	float historySum;
    if( zBasePositionHistoryBuffer.is_full() ) {
        historySum = zBasePositionHistoryBuffer.front();
    }
	else{
        historySum = baseZPosition;
    }
    zErrorPosition = (baroAltitude * 100.0F) - (historySum + zPositionCorrection);
}

void computeVelocity(float filteredAccelZ, float dt)
{
	filteredAccelZ -= accelOneG;
	filteredAccelZ *= 100.0F;
	
	accelZCorrection += zErrorPosition * k3_z  * dt;
	currentComputedZVelocity += zErrorPosition * k2_z  * dt;
	zPositionCorrection += zErrorPosition * k1_z  * dt;
		
	float velocityIncrease = (filteredAccelZ + accelZCorrection) * dt;
	baseZPosition += (currentComputedZVelocity + velocityIncrease*0.5) * dt;
	currentComputedZVelocity += velocityIncrease;
	
	computedZVelocity = currentComputedZVelocity;

	zBasePositionHistoryBuffer.push_back(baseZPosition);
}

#endif
