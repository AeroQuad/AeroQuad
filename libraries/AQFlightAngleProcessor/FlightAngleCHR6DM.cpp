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

#include "FlightAngleCHR6DM.h"

FlightAngleCHR6DM::FlightAngleCHR6DM(CHR6DM chr6dm, Gyroscope *gyro,Accelerometer *accel) : FlightAngleProcessor(gyro,accel) 
{
  _chr6dm = &chr6dm;
}

void FlightAngleCHR6DM::initialize() 
{
  calibrate();
}

void FlightAngleCHR6DM::calculate(unsigned long G_Dt) 
{   
  _angle[ROLL]  =  _chr6dm->data.roll - _zeroRoll;
  _angle[PITCH] =  _chr6dm->data.pitch - _zeroPitch;
  _chr6dm->CHR_RollAngle = _angle[ROLL]; //ugly since gotta access through accel class
  _chr6dm->CHR_PitchAngle = _angle[PITCH];
}
  
void FlightAngleCHR6DM::calibrate() 
{
  _zeroRoll = _chr6dm->data.roll;
  _zeroPitch = _chr6dm->data.pitch;
}
  


