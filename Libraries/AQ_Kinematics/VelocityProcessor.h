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
#include "AP_Buffer.h"

float k1_z = 0.0;                      
float k2_z = 0.0;                      
float k3_z = 0.0;              
//float accelZVelocity = 0.0;    
float timeConstantZ = 0.1;    
//float accelZPositionError = 0.0;
//float accelZCorrection = 0.0;
//float accelZVelocityIncrease = 0.0;

void initVelocityProcessor();
void updateVelocityProcessorGains();
void computeVelocity(float filteredAccelZ, float dt);
void computeVelocityErrorWithBaroAltitude(float baroAltidude);

void initVelocityProcessor()
{
	updateVelocityProcessorGains();
}

void updateVelocityProcessorGains()
{
    if( timeConstantZ == 0 ) {
        k1_z = k2_z = k3_z = 0;
    }else{
        k1_z = 3 / timeConstantZ;
        k2_z = 3 / (timeConstantZ*timeConstantZ);
        k3_z = 1 / (timeConstantZ*timeConstantZ*timeConstantZ);
    }
}

//float _velocity = 0.0;
float _velocity_z = 0.0;
float _position_error_z = 0.0;
float _position_correction_z = 0.0;
float accel_correction_ef_z = 0.0;
float velocityPreviousBaroAltitude = 0.0;
float _position_base_z = 0.0;
float hist_position_base_z = 0.0;
AP_BufferFloat_Size15   _hist_position_estimate_z;

void computeVelocityErrorWithBaroAltitude(float baroAltidude)
{
	hist_position_base_z = _hist_position_estimate_z.peek(14);
    // calculate error in position from baro with our estimate
    _position_error_z = baroAltidude - (hist_position_base_z + _position_correction_z);
	velocityPreviousBaroAltitude = baroAltidude;
}


void computeVelocity(float filteredAccelZ, float dt)
{
	accel_correction_ef_z += _position_error_z * k3_z  * dt;
	
	_velocity_z += _position_error_z * k2_z  * dt;
	
	_position_correction_z += _position_error_z * k1_z  * dt;
	
	float velocity_increase = (filteredAccelZ + accel_correction_ef_z) * dt;
	
	_position_base_z += (_velocity_z + velocity_increase*0.5) * dt;
	
	_velocity_z += velocity_increase;
	
	_hist_position_estimate_z.add(_position_base_z);
	
	Serial.println(_velocity_z);
}



#endif
