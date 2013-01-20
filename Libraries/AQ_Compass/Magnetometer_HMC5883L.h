/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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


#ifndef _AEROQUAD_MAGNETOMETER_HMC5883L_H_
#define _AEROQUAD_MAGNETOMETER_HMC5883L_H_

#include "Compass.h"

#include "Arduino.h"

#include "Magnetometer_HMC58xx.h"

void readSpecificMag(float *rawMag) {

  #if defined(SPARKFUN_9DOF_5883L)
    // JI - 1/4/12 - Add _5883L to define
    // JI - 11/24/11 - SparkFun DOF on v2p1 Shield Configuration
    // JI - 11/24/11 - 5883L X axis points aft
    // JI - 11/24/11 - 5883L Sensor Orientation 3
    rawMag[XAXIS] = -readShortI2C();
    rawMag[ZAXIS] = -readShortI2C();
    rawMag[YAXIS] =  readShortI2C();
  #elif defined(SPARKFUN_5883L_BOB)
    // JI - 11/24/11 - Sparkfun 5883L Breakout Board Upside Down on v2p0 shield
    // JI - 11/24/11 - 5883L is upside down, X axis points forward
    // JI - 11/24/11 - 5883L Sensor Orientation 5
    rawMag[XAXIS] = readShortI2C();
    rawMag[ZAXIS] = readShortI2C();
    rawMag[YAXIS] = readShortI2C();
  #elif defined (HMC5883L)  // baloo
    rawMag[YAXIS] =  readShortI2C();
    rawMag[ZAXIS] = -readShortI2C();
    rawMag[XAXIS] =  readShortI2C();
  #else 
    #error Define HMC5883L Orientation
  #endif
}

#endif
