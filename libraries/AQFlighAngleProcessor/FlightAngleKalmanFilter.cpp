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


#include "FlightAngleKalmanFilter.h"

FlightAngleKalmanFilter::FlightAngleKalmanFilter()
{
  for (byte axis = ROLL; axis < YAW; axis ++) 
  {
    x_angle[axis] = 0;
    x_bias[axis] = 0;
    P_00[axis] = 0;
    P_01[axis] = 0;
    P_10[axis] = 0;
    P_11[axis] = 0;
  }
  _type = KF;
}

void FlightAngleKalmanFilter::initialize(void) 
{
  Q_angle = 0.001;
  Q_gyro = 0.003;
  R_angle = 0.03;
}

void FlightAngleKalmanFilter::calculate(float G_Dt) 
{
  _angle[ROLL] = _calculate(ROLL, _accel->angleDeg(ROLL), _gyro->rateDegPerSec(ROLL), G_Dt);
  _angle[PITCH] = _calculate(PITCH, _accel->angleDeg(PITCH), _gyro->rateDegPerSec(PITCH), G_Dt);
}

float FlightAngleKalmanFilter::getGyroUnbias(byte axis) 
{
  return _gyro->getFlightData(axis);
}


float FlightAngleKalmanFilter::_calculate(byte axis, float newAngle, float newRate, float G_Dt) 
{
  x_angle[axis] += G_Dt * (newRate - x_bias[axis]);
  P_00[axis] +=  - G_Dt * (P_10[axis] + P_01[axis]) + Q_angle * G_Dt;
  P_01[axis] +=  - G_Dt * P_11[axis];
  P_10[axis] +=  - G_Dt * P_11[axis];
  P_11[axis] +=  + Q_gyro * G_Dt;
  
  y = newAngle - x_angle[axis];
  S = P_00[axis] + R_angle;
  K_0 = P_00[axis] / S;
  K_1 = P_10[axis] / S;
  
  x_angle[axis] +=  K_0 * y;
  x_bias[axis]  +=  K_1 * y;
  P_00[axis] -= K_0 * P_00[axis];
  P_01[axis] -= K_0 * P_01[axis];
  P_10[axis] -= K_1 * P_00[axis];
  P_11[axis] -= K_1 * P_01[axis];
  
  return x_angle[axis];
}



