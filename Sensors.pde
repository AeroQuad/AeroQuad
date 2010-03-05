/*
  AeroQuad v1.6 - March 2010
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

#include "Sensors.h"

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
  for (axis = ROLL; axis < YAW; axis++) {
    for (int i=0; i<FINDZERO; i++)
      findZero[i] = analogRead(accelChannel[axis]);
    accelZero[axis] = findMode(findZero, FINDZERO);
  }
  accelZero[ZAXIS] = ZMAX - ((ZMAX - ZMIN)/2);
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
  return (gyroADC[axis] / 1024) * aref / gyroScaleFactor;
}

float rateRadPerSec(byte axis) {
  return radians((gyroADC[axis] / 1024) * aref / gyroScaleFactor);
}

float angleDeg(byte axis) {
  return degrees(angleRad(axis));
} 

float angleRad(byte axis) {
  if (axis == PITCH) return arctan2(accelADC[PITCH], sqrt((accelADC[ROLL] * accelADC[ROLL]) + (accelADC[ZAXIS] * accelADC[ZAXIS])));
  if (axis == ROLL) return arctan2(accelADC[ROLL], sqrt((accelADC[PITCH] * accelADC[PITCH]) + (accelADC[ZAXIS] * accelADC[ZAXIS])));
}

/*float angleDegRoll(void) {
  return arctan2(accelADC[ROLL], sqrt((accelADC[PITCH] * accelADC[PITCH]) + (accelADC[ZAXIS] * accelADC[ZAXIS]))) * 57.2957795;
}

float angleDegPitch(void) {
  return arctan2(accelADC[PITCH], sqrt((accelADC[ROLL] * accelADC[ROLL]) + (accelADC[ZAXIS] * accelADC[ZAXIS]))) * 57.2957795;
}*/

