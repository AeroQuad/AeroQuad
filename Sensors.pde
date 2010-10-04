/*
  AeroQuad v2.1 - October 2010
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

// Sensors.pde is responsible for taking on board sensor measuremens of the AeroQuad

void readSensors(void) {
  // *********************** Read Critical Sensors **********************
  // Apply low pass filter to sensor values and center around zero
  gyro.measure(); // defined in Gyro.h
  accel.measure(); // defined in Accel.h
 
 // ********************* Read Slower Sensors *******************
 #if defined(AeroQuadMega_v2)
   if (currentTime > (compassTime + COMPASSLOOPTIME)) { // 10Hz
     compass.measure(); // defined in compass.h
     compassTime = currentTime;
   }
 #endif
 #if defined(AeroQuadMega_v2) || defined(ArduCopter)
   if (currentTime > (altitudeTime + ALTITUDELOOPTIME)) { // 37Hz
     altitude.measure(); // defined in altitude.h
     altitudeTime = currentTime;
   }
 #endif
  
 // ****************** Calculate Absolute Angle *****************
 flightAngle.calculate(); // defined in FlightAngle.h
}

// Alternate method to calculate arctangent from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
float arctan2(float y, float x) {
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

// Used for sensor calibration
// The mode of a set of numbers returns the value that occurs most frequently
int findMode(int *data, int arraySize) {
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
