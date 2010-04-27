/*
  AeroQuad v1.8 - April 2010
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
#define AZPIN 12 // Auto zero pin for IDG500 gyros
#define ZAXIS 2

int gyroChannel[3] = {ROLLRATEPIN, PITCHRATEPIN, YAWRATEPIN};
int accelChannel[3] = {ROLLACCELPIN, PITCHACCELPIN, ZACCELPIN};

// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
// If you don't have a DMM use the following:
// AeroQuad Shield v1.7, aref = 3.0
// AeroQuad Shield v1.6 or below, aref = 2.8
float aref; // Read in from EEPROM
int axis;

// Accelerometer Values
// Output is ratiometric for ADXL 335
// If Vs = 3.6V, then output sensitivity is 360mV/g
// If Vs = 2V, then it's 195 mV/g
// Then if Vs = 3.3V, then it's 329.062 mV/g
float accelScaleFactor = 0.000329062;

// Gyro Values
// Update these variables if using a different gyro
float gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
float gyroScaleFactor = 0.002;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec)

// Accelerometer setup
int accelData[3] = {0,0,0};
int accelZero[3] = {0,0,0};
int accelADC[3] = {0,0,0};

// Gyro setup
int gyroData[3] = {0,0,0};
int gyroZero[3] = {0,0,0};
int gyroADC[3] = {0,0,0};

// Calibration parameters
#define ZEROLIMIT 2
#define FINDZERO 50
int findZero[FINDZERO];


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
