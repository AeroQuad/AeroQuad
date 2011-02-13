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

#ifndef _AQ_FLIGHT_ANGLE_COMP_FILTER_H_
#define _AQ_FLIGHT_ANGLE_COMP_FILTER_H_

#include "FlightAngleProcessor.h"

class FlightAngleCompFilter : public FlightAngleProcessor 
{
private:
  float _previousAngle[2];
  float _filterTerm0[2];
  float _filterTerm1[2];
  float _filterTerm2[2];
  float _timeConstantCF;

  void _initialize(byte axis) 
  {
    _previousAngle[axis] = _accel->angleDeg(axis);
    _filterTerm2[axis] = _gyro->rateDegPerSec(axis);
    _timeConstantCF = _timeConstant; // timeConstant is a global variable read in from EEPROM
    // timeConstantCF should have been read in from set method, but needed common way for CF and KF to be initialized
    // Will take care of better OO implementation in future revision
  }
  
  float _calculate(byte axis, float newAngle, float newRate) 
  {
    _filterTerm0[axis] = (newAngle - _previousAngle[axis]) * _timeConstantCF *  _timeConstantCF;
    _filterTerm2[axis] += _filterTerm0[axis] * G_Dt;
    _filterTerm1[axis] = _filterTerm2[axis] + (newAngle - _previousAngle[axis]) * 2 *  _timeConstantCF + newRate;
    _previousAngle[axis] = (_filterTerm1[axis] * G_Dt) + _previousAngle[axis];
    return _previousAngle[axis]; // This is actually the current angle, but is stored for the next iteration
  }

public:
  FlightAngleCompFilter() : FlightAngleProcessor() 
  {
    _filterTerm0[ROLL] = 0;
    _filterTerm1[ROLL] = 0;
    _filterTerm0[PITCH] = 0;
    _filterTerm1[PITCH] = 0;
    _type = CF;
  }
  
  void initialize() 
  {
    for (byte axis = ROLL; axis < YAW; axis++)
    {
      _initialize(axis);
    }
  }
  
  void calculate() 
  {
    _angle[ROLL] = _calculate(ROLL, _accel->angleDeg(ROLL), _gyro->rateDegPerSec(ROLL));
    _angle[PITCH] = _calculate(PITCH, _accel->angleDeg(PITCH), _gyro->rateDegPerSec(PITCH));
  }
  
  float getGyroUnbias(byte axis) 
  {
    return _gyro->getFlightData(axis);
  }
  
  void calibrate() {}
};

#endif