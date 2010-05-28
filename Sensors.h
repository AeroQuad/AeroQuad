/*
  AeroQuad v1.8 - May 2010
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
// Update these variables if using a different accel
// Output is ratiometric for ADXL 335
// If Vs = 3.6V, then output sensitivity is 360mV/g
// If Vs = 2V, then it's 195 mV/g
// Then if Vs = 3.3V, then it's 329.062 mV/g
float accelScaleFactor = 0.000329062;

// Gyro Values
// Update these variables if using a different gyro
float gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
float gyroScaleFactor = 0.002;       // IDG/IXZ500 sensitivity = 2mV/(deg/sec)

// Adjust for gyro drift
// http://aeroquad.com/entry.php?4-
int lastAccel[2]={0,0};
long accelAge[2]={0,0};
int positiveGyroCount[2]={1,1};
int negativeGyroCount[2]={1,1};
int zeroGyroCount[2]={1,1};

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


int findMode(int *data, int arraySize) {
  // The mode of a set of numbers is the value that occurs most frequently
  boolean done = 0;
  byte i;
  int temp, maxData, frequency, maxFrequency;
  
  // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (i=0; i<(arraySize-1); i++) {
      if (data[i] > data[i+1]) {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }
  
  temp = 0;
  frequency = 0;
  maxFrequency = 0;
  
  // Count number of times a value occurs in sorted array
  for (i=0; i<arraySize; i++) {
    if (data[i] > temp) {
      frequency = 0;
      temp = data[i];
      frequency++;
    } else if (data[i] == temp) frequency++;
    if (frequency > maxFrequency) {
      maxFrequency = frequency;
      maxData = data[i];
    }
  }
  return maxData;
}

// Allows user to zero gyros on command
void zeroGyros() {
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    for (int i=0; i<FINDZERO; i++)
      findZero[i] = analogRead(gyroChannel[axis]);
    gyroZero[axis] = findMode(findZero, FINDZERO);
  }
  writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
}

void autoZeroGyros() {
  digitalWrite(AZPIN, HIGH);
  delayMicroseconds(750);
  digitalWrite(AZPIN, LOW);
  delay(8);
}

// Allows user to zero accelerometers on command
void zeroAccelerometers() {
  // Z accel zero point is determined in calibration through Configurator
  for (axis = ROLL; axis < YAW; axis++) {
    for (int i=0; i<FINDZERO; i++)
      findZero[i] = analogRead(accelChannel[axis]);
    accelZero[axis] = findMode(findZero, FINDZERO);
  }
  accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
  writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
  writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
}

// Works faster and is smaller than the constrain() function
int limitRange(int data, int minLimit, int maxLimit) {
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}

float arctan2(float y, float x) {
  // Taken from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
   float coeff_1 = PI/4;
   float coeff_2 = 3*coeff_1;
   float abs_y = abs(y)+1e-10;      // kludge to prevent 0/0 condition
   float r, angle;
   
   if (x >= 0) {
     r = (x - abs_y) / (x + abs_y);
     angle = coeff_1 - coeff_1 * r;
   }
   else {
     r = (x + abs_y) / (abs_y - x);
     angle = coeff_2 - coeff_1 * r;
   }
   if (y < 0)
     return(-angle);     // negate if in quad III or IV
   else
     return(angle);
}

float rateDegPerSec(byte axis) {
  return (gyroADC[axis] / 1024.0) * aref / gyroScaleFactor;
}

float rateRadPerSec(byte axis) {
  return radians((gyroADC[axis] / 1024.0) * aref / gyroScaleFactor);
}

float angleRad(byte axis) {
  if (axis == PITCH) return arctan2(accelADC[PITCH], sqrt((long(accelADC[ROLL]) * accelADC[ROLL]) + (long(accelADC[ZAXIS]) * accelADC[ZAXIS])));
  if (axis == ROLL) return arctan2(accelADC[ROLL], sqrt((long(accelADC[PITCH]) * accelADC[PITCH]) + (long(accelADC[ZAXIS]) * accelADC[ZAXIS])));
}
float angleDeg(byte axis) {
  return degrees(angleRad(axis));
} 

#endif
