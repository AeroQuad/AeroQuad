/*
  AeroQuad v1.0 - April 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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
  filterTerm[0] = timeConstant / (timeConstant + 0.010); //10ms = ESC update rate
  filterTerm[1] = 1 - filterTerm[0];
}

float filterData(float previousAngle, int gyroADC, int accelADC, float *filterTerm, float dt) {
  // For Sparkfun 5DOF IMU
  // accelerometerOutput = (N-512)/1024*(double)10.78;
  // gyroOutput = (N-512)/1024*(double)28.783;
  return (filterTerm[0] * (previousAngle + (gyroADC / 1024.0 * 28.783 * dt))) + (filterTerm[1] * (accelADC / 1024.0 * 10.78)) * 57.2957795;
}

int smooth(int currentData, int previousData, float smoothFactor) {
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}
