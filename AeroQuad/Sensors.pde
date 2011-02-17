/*
  AeroQuad v2.1 - January 2011
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

// Sensors.pde is responsible for taking on board sensor measuremens of the AeroQuad

void readSensors(void) {
  // *********************** Read Critical Sensors **********************
  // Apply low pass filter to sensor values and center around zero
  rateGyro.measure(); // defined in RateGyro.h
  accel.measure(); // defined in Accel.h
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    compass.measure();
  #endif
 
  // ********************* Read Slower Sensors *******************
  #if defined(COMPASS_INSTALLED)
    if (currentTime > compassTime) {
      compass.measure(kinematics.getAttitude(ROLL), kinematics.getAttitude(PITCH)); // defined in compass.h
      compassTime = currentTime + COMPASSLOOPTIME;
    }
  #endif
  #if defined(AltitudeHold)
    if (currentTime > altitudeTime) {
      altitude.measure(); // defined in altitude.h
      altitudeTime = currentTime + ALTITUDELOOPTIME;
    }
  #endif
  #if defined(BattMonitor)
    if (currentTime > batteryTime) {
      batteryMonitor.measure(armed);
      batteryTime = currentTime + BATTERYLOOPTIME;
    }
  #endif
  
  #if defined(COMPASS_INSTALLED)
    kinematics.calculate(rateGyro.getNonDriftCorrectedRate(ROLL),  \
                         rateGyro.getNonDriftCorrectedRate(PITCH), \
                         rateGyro.getNonDriftCorrectedRate(YAW),   \
                         accel.getData(XAXIS),                     \
                         accel.getData(YAXIS),                     \
                         accel.getData(ZAXIS),                     \
                         accel.getOneG(),                          \
                         compass.getHdgXY(XAXIS),                  \
                         compass.getHdgXY(YAXIS));
  #else
    kinematics.calculate(rateGyro.getNonDriftCorrectedRate(ROLL),  \
                         rateGyro.getNonDriftCorrectedRate(PITCH), \
                         rateGyro.getNonDriftCorrectedRate(YAW),   \
                         accel.getData(XAXIS),                     \
                         accel.getData(YAXIS),                     \
                         accel.getData(ZAXIS),                     \
                         accel.getOneG(),                          \
                         0.0,                                      \
                         0.0);
  
  #endif
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
// Takes the median of 50 results as zero
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
float findMode(float *data, int arraySize) {
  float temp;
#else
int findMode(int *data, int arraySize) {                  //Thanks ala42! Post: http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code/page5
  int temp;
#endif
  boolean done = 0;
  byte i;
  
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
  
  return data[arraySize/2]; // return the median value
}
