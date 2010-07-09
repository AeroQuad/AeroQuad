/*
  AeroQuad v2.0 - July 2010
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

void readSensors(void) {
  // *********************** Read Sensors **********************
  // Apply low pass filter to sensor values and center around zero
  gyro.measure(); // defined in Gyro.h
  accel.measure(); // defined in Accel.h
 
  #ifdef OriginalIMU
    gyro.invert(ROLL);
    gyro.invert(PITCH);
  #endif
  #if defined (IDG) | defined (APM) | defined (AeroQuad_v2)
    gyro.invert(YAW);
  #endif
  #ifdef AeroQuad_Wii
    accel.invert(ROLL);
  #endif

 // ************ Correct for gyro drift by FabQuad **************
 // ************ http://aeroquad.com/entry.php?4-  **************
 /*for (axis = ROLL; axis < YAW; axis++) {              
   if (abs(lastAccel[axis]-accel.getData(axis)) < 5) { // if accel is same as previous cycle
     accelAge[axis]++;
     if (accelAge[axis] >= 4) {  // if accel was the same long enough, we can assume that there is no (fast) rotation
       if (gyro.getData(axis) < 0) { 
         negativeGyroCount[axis]++; // if gyro still indicates negative rotation, that's additional signal that gyrozero is too low
       } else if (gyro.getData(axis) > 0) {
         positiveGyroCount[axis]++;  // additional signal that gyrozero is too high
       } else {
         zeroGyroCount[axis]++; // additional signal that gyrozero is correct
       }
       accelAge[axis]=0;
       if (zeroGyroCount[axis] + negativeGyroCount[axis] + positiveGyroCount[axis] > 200) {
         if (negativeGyroCount[axis] >= 1.3*(zeroGyroCount[axis]+positiveGyroCount[axis])) gyro.setZero(axis, gyro.getZero(axis) + 1);  // enough signals the gyrozero is too low
         if (positiveGyroCount[axis] >= 1.3*(zeroGyroCount[axis]+negativeGyroCount[axis])) gyro.setZero(axis, gyro.getZero(axis) - 1);  // enough signals the gyrozero is too high
         zeroGyroCount[axis]=0;
         negativeGyroCount[axis]=0;
         positiveGyroCount[axis]=0;
       }
     }
   } else { // accel different, restart
     lastAccel[axis]=accel.getData(axis);
     accelAge[axis]=0;
   }
 }*/

 // ****************** Calculate Absolute Angle *****************
 // angle[axis].calculate() defined in FlightAngle.h
 flightAngle[ROLL] = angle[ROLL].calculate(accel.angleDeg(ROLL), gyro.rateDegPerSec(ROLL));
 flightAngle[PITCH] = angle[PITCH].calculate(accel.angleDeg(PITCH), gyro.rateDegPerSec(PITCH));
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
    
    temp = -32768;
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
