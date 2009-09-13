/*
  AeroQuad v1.3.2 - September 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
  
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

void configureFilter(float timeConstant) {
  flightAngle[ROLL] = atan2(analogRead(accelChannel[ROLL]) - accelZero[ROLL], analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS]) * 57.2957795;
  flightAngle[PITCH] = atan2(analogRead(accelChannel[PITCH]) - accelZero[PITCH], analogRead(accelChannel[ZAXIS]) - accelZero[ZAXIS]) * 57.2957795;
  filterTermRoll[2] = -(analogRead(gyroChannel[ROLL]) - gyroZero[ROLL]) / 29473.792 * 57.2957795;
  filterTermPitch[2] = -(analogRead(gyroChannel[PITCH]) - gyroZero[PITCH]) / 29473.792 * 57.2957795;
}

float filterData(float previousAngle, int gyroADC, float angle, float *filterTerm, float dt) {
  // Written by RoyLB at:
  // http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286
  static float filter;
  float accel, gyro;
  float aref = 3.0;
  
  // For Sparkfun 5DOF IMU
  // accelerometerOutput = (N-512)/1024*(double)10.78; (rad)
  // gyroOutput = (N-512)/1024*(double)28.783; (rad/sec)
  accel = angle * 57.2957795;
  //gyro = gyroADC / 29473.792 * 57.2957795;
  gyro = (gyroADC / 1024) * aref / 0.002;
  
  ///////////////////////
  // constants or parameters:
  // timeConstant - bandwidth of filter (1/sec). Need to tune this to match sensor performance.
  // T - iteration rate of the filter (sec)
  ///////////////////////
  // variables:
  // int_x1 (filterTerm[0]) - input to the first integrator (deg/sec/sec)
  // int_x2 (filterTerm[1]) - input to the second integrator (deg/sec)
  // int_y1 (filterTerm[2]) - output of the first integrator (deg/sec). This needs to be saved each iteration
  //////////////////////
  // inputs:
  // gyro - gyro output (deg/sec)
  // accel - accelerometer input to filter (deg)
  // x_accel - accelerometer output in x-axis (g)
  // z_accel - accelerometer output in z-axis (g)
  // accel_ang - derived angle based on arctan(x_accel,z_accel), (deg)
  //////////////////////
  // outputs:
  // filter - complementary filter output (and output of second integrator), (deg)
  //            This also needs to be saved each iteration.
  ///////////////////////

  filterTerm[0] = (accel - previousAngle) * timeConstant * timeConstant;
  filterTerm[2] = (dt * filterTerm[0]) + filterTerm[2];
  filterTerm[1] = filterTerm[2] + (accel - previousAngle) * 2 * timeConstant + gyro;
  return (dt * filterTerm[1]) + previousAngle;
}

int smooth(int currentData, int previousData, float smoothFactor) {
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}
