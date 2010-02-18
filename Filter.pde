/*
  AeroQuad v1.6 - February 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
  
  Second order complementary filter written by LB Roy
  http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
 
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

#include "Filter.h"

void configureFilter(float timeConstant) {
  #ifndef KalmanFilter
    flightAngle[ROLL] = angleDeg(ROLL);
    flightAngle[PITCH] = angleDeg(PITCH);
    filterTermRoll[2] = -rateDegPerSec(ROLL);
    filterTermPitch[2] = -rateDegPerSec(PITCH);
  #endif
  #ifdef KalmanFilter
    // These parameters need to be further optimized
    initGyro1DKalman(&rollFilter, 0.001, 0.003, 0.03);
    initGyro1DKalman(&pitchFilter, 0.001, 0.003, 0.03);
  #endif
}


#ifndef KalmanFilter
float filterData(float previousAngle, float newAngle, float rate , float *filterTerm, float dt) {
  // Written by RoyLB at:
  // http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
  filterTerm[0] = (newAngle - previousAngle) * timeConstant * timeConstant;
  filterTerm[2] = (dt * filterTerm[0]) + filterTerm[2];
  filterTerm[1] = filterTerm[2] + (newAngle - previousAngle) * 2 * timeConstant + rate;
  return (dt * filterTerm[1]) + previousAngle;
}
#endif

#ifdef KalmanFilter
// The Kalman filter implementation is directly taken from the work
// of Tom Pycke at: http://tom.pycke.be/mav/90/sparkfuns-5dof
void initGyro1DKalman(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle) {
	filterdata->Q_angle = Q_angle;
	filterdata->Q_gyro  = Q_gyro;
	filterdata->R_angle = R_angle;
}

void predictKalman(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt) {
	filterdata->x_angle += dt * (dotAngle - filterdata->x_bias);
	filterdata->P_00 +=  - dt * (filterdata->P_10 + filterdata->P_01) + filterdata->Q_angle * dt;
	filterdata->P_01 +=  - dt * filterdata->P_11;
	filterdata->P_10 +=  - dt * filterdata->P_11;
	filterdata->P_11 +=  + filterdata->Q_gyro * dt;
}

float updateKalman(struct Gyro1DKalman *filterdata, const float angle_m) {
	const float y = angle_m - filterdata->x_angle;
	const float S = filterdata->P_00 + filterdata->R_angle;
	const float K_0 = filterdata->P_00 / S;
	const float K_1 = filterdata->P_10 / S;
	
	filterdata->x_angle +=  K_0 * y;
	filterdata->x_bias  +=  K_1 * y;
	filterdata->P_00 -= K_0 * filterdata->P_00;
	filterdata->P_01 -= K_0 * filterdata->P_01;
	filterdata->P_10 -= K_1 * filterdata->P_00;
	filterdata->P_11 -= K_1 * filterdata->P_01;

	return filterdata->x_angle;
}
#endif

int smooth(int currentData, int previousData, float smoothFactor) {
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}
