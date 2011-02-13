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

#include "FlightAngleCompFilter.h"

FlightAngleCompFilter::FlightAngleCompFilter(Gyroscope *gyro,Accelerometer *accel) : FlightAngleProcessor(gyro,accel) 
{
  _filterTerm0[ROLL] = 0;
  _filterTerm1[ROLL] = 0;
  _filterTerm0[PITCH] = 0;
  _filterTerm1[PITCH] = 0;
  _type = CF;
}

void FlightAngleCompFilter::_initialize(byte axis) 
{
  _previousAngle[axis] = _accel->angleDeg(axis);
  _filterTerm2[axis] = _gyro->rateDegPerSec(axis);
//  _timeConstantCF = _timeConstant; // timeConstant is a global variable read in from EEPROM
  _timeConstantCF = 7.0; // timeConstant is a global variable read in from EEPROM
  // timeConstantCF should have been read in from set method, but needed common way for CF and KF to be initialized
  // Will take care of better OO implementation in future revision
}
  
float FlightAngleCompFilter::_calculate(byte axis, float newAngle, float newRate,unsigned long G_Dt) 
{
  _filterTerm0[axis] = (newAngle - _previousAngle[axis]) * _timeConstantCF *  _timeConstantCF;
  _filterTerm2[axis] += _filterTerm0[axis] * G_Dt;
  _filterTerm1[axis] = _filterTerm2[axis] + (newAngle - _previousAngle[axis]) * 2 *  _timeConstantCF + newRate;
  _previousAngle[axis] = (_filterTerm1[axis] * G_Dt) + _previousAngle[axis];
  return _previousAngle[axis]; // This is actually the current angle, but is stored for the next iteration
}
  
void FlightAngleCompFilter::initialize() 
{
  for (byte axis = ROLL; axis < YAW; axis++)
  {
    _initialize(axis);
  }
}
  
void FlightAngleCompFilter::calculate(unsigned long G_Dt) 
{
  _angle[ROLL] = _calculate(ROLL, _accel->angleDeg(ROLL), _gyro->rateDegPerSec(ROLL),G_Dt);
  _angle[PITCH] = _calculate(PITCH, _accel->angleDeg(PITCH), _gyro->rateDegPerSec(PITCH),G_Dt);
}
  
 