/*
  AeroQuad v1.6 - February 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

#ifndef SENSORS_H
#define SENSORS_H

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5
int gyroChannel[3] = {ROLLRATEPIN, PITCHRATEPIN, YAWRATEPIN};
int accelChannel[3] = {ROLLACCELPIN, PITCHACCELPIN, ZACCELPIN};

// These A/D values depend on how well the sensors are mounted
// change these values to your unique configuration
// #define XMIN 405
// #define XMAX 607
// #define YMIN 409
// #define YMAX 618
#define ZMIN 479
#define ZMAX 715
#define ZAXIS 2
#define ZEROLIMIT 2
int axis;

// Accelerometer setup
int accelData[3] = {0,0,0};
int accelZero[3] = {0,0,0};
int accelADC[3] = {0,0,0};

// Gyro setup
int gyroData[3] = {0,0,0};
int gyroZero[3] = {0,0,0};
int gyroADC[3] = {0,0,0};

// Calibration parameters
#define FINDZERO 10
int findZero[FINDZERO];

// Analog Reference Value
float aref = 2.8; // Measured with a DMM
float gyroScaleFactor = 0.002; // From datasheet

int findMode(int *data, int arraySize);
void zeroGyros();
void autoZeroGyros();
void zeroAccelerometers();
int limitRange(int data, int minLimit, int maxLimit);
float rateDegPerSec(byte axis);
float rateRadPerSec(byte axis);
float angleDeg(byte axis);
float angleRad(byte axis);

#endif
