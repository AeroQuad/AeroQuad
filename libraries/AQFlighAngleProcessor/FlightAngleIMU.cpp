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


#include "FlightAngleIMU.h"

void FlightAngleIMU::initialize(void) 
{
  // estimated orientation quaternion elements with initial conditions
  SEq_1 = 1.0f; // w
  SEq_2 = 0.0f; // x
  SEq_3 = 0.0f; // y
  SEq_4 = 0.0f; // z
}
  
void FlightAngleIMU::calculate(float G_Dt) 
{
  filterUpdate(_gyro->rateRadPerSec(ROLL), _gyro->rateRadPerSec(PITCH), _gyro->rateRadPerSec(YAW), _accel->getRaw(XAXIS), _accel->getRaw(YAXIS), _accel->getRaw(ZAXIS), G_Dt);
  _angle[ROLL] = degrees(-asin((2 * SEq_2 * SEq_4) + (2 * SEq_1 * SEq_3)));
  _angle[PITCH] = degrees(atan2((2 * SEq_3 * SEq_4) - (2 *SEq_1 * SEq_2), (2 * SEq_1 * SEq_1) + (2 *SEq_4 * SEq_4) - 1));
  _angle[YAW] = degrees(atan2((2 * SEq_2 * SEq_3) - (2 * SEq_1 * SEq_4), (2 * SEq_1 * SEq_1) + (2 * SEq_2 * SEq_2) -1));
}
  
float FlightAngleIMU::getGyroUnbias(byte axis) 
{
  return _gyro->getFlightData(axis);
}

void FlightAngleIMU::filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float G_Dt) 
{
  // Local system variables
  float norm; // vector norm
  float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
  float f_1, f_2, f_3; // objective function elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
  // Axulirary variables to avoid repeated calcualtions
  float halfSEq_1 = 0.5f * SEq_1;
  float halfSEq_2 = 0.5f * SEq_2;
  float halfSEq_3 = 0.5f * SEq_3;
  float halfSEq_4 = 0.5f * SEq_4;
  float twoSEq_1 = 2.0f * SEq_1;
  float twoSEq_2 = 2.0f * SEq_2;
  float twoSEq_3 = 2.0f * SEq_3;
  // Normalise the accelerometer measurement
  norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x /= norm;
  a_y /= norm;
  a_z /= norm;
  // Compute the objective function and Jacobian
  f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
  f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
  f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
  J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
  J_12or23 = 2.0f * SEq_4;
  J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
  J_14or21 = twoSEq_2;
  J_32 = 2.0f * J_14or21; // negated in matrix multiplication
  J_33 = 2.0f * J_11or24; // negated in matrix multiplication
  // Compute the gradient (matrix multiplication)
  SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
  SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
  SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
  SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
  // Normalise the gradient
  norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
  SEqHatDot_1 /= norm;
  SEqHatDot_2 /= norm;
  SEqHatDot_3 /= norm;
  SEqHatDot_4 /= norm;
  // Compute the quaternion derrivative measured by gyroscopes
  SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
  SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
  SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
  SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
  // Compute then integrate the estimated quaternion derrivative
  SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * G_Dt;
  SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * G_Dt;
  SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * G_Dt;
  SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * G_Dt;
  // Normalise quaternion
  norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
  SEq_1 /= norm;
  SEq_2 /= norm;
  SEq_3 /= norm;
  SEq_4 /= norm;
}
  

 
