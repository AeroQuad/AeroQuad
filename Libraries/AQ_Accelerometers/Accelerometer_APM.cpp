/*
  AeroQuad v3.0 - February 2011
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

#include "Accel_APM.h"

Accel_APM::Accel_APM() {
  accelScaleFactor = G_2_MPS2((3.3/4096) / 0.330);
}

void Accel_APM::initialize(void) { 
  //initializeApmADC();
}

void Accel_APM::measure(void) {
  // The following 3 lines marked with ** read the accelerometer and assign it's 
  // data to accelRaw in the correct order and phase to suit the standard shield 
  // installation orientation.  See TBD for details.  If your shield is not
  //  installed in this orientation, this is where you make the changes.
  //accelRaw[XAXIS] = readApmADC(XAXIS + 3);
  if (accelRaw[XAXIS] > 500)
    accelRaw[XAXIS] = accelRaw[XAXIS] - accelZero[XAXIS];  // **
    
  //accelRaw[YAXIS] = readApmADC(YAXIS + 3);
  if (accelRaw[YAXIS] > 500)
    accelRaw[YAXIS] = accelZero[YAXIS] - accelRaw[YAXIS];  // **
    
  //accelRaw[ZAXIS] = readApmADC(ZAXIS + 3);
  if (accelRaw[ZAXIS] > 500)
    accelRaw[ZAXIS] = accelZero[ZAXIS] - accelRaw[ZAXIS];  // **
  
    //for (byte axis = XAXIS; axis < LASTAXIS; axis++)
      //accelVector[axis] = smooth(accelRaw[axis] * accelScaleFactor, accelVector[axis], smoothFactor);
 }
 
 void Accel_APM::calibrate(void) {
  int findZero[FINDZERO];
  
  for(byte calAxis = 0; calAxis < LASTAXIS; calAxis++) {
    for (int i=0; i<FINDZERO; i++) {
      //findZero[i] = readApmADC(calAxis + 3);
      delay(10);
    }
    accelZero[calAxis] = findMedian(findZero, FINDZERO);
  }

  // replace with estimated Z axis 0g value
  accelZero[ZAXIS] = (accelZero[XAXIS] + accelZero[YAXIS]) / 2;
  
  // store accel value that represents 1g
  measure();
  accelOneG = -accelVector[ZAXIS];
}

/*void Accel_APM::zero() {
  for (byte n = 3; n < 6; n++) {
    adc_value[n] = 0;
    adc_counter[n] = 0;
  }
}*/