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

// Class to define sensors that can determine altitude

// ***********************************************************************
// ************************** BarometricSensor Class *********************
// ***********************************************************************

#ifndef _BAROMETRIC_SENSOR_BMP085_H
#define _BAROMETRIC_SENSOR_BMP085_H

#include <BarometricSensor.h>

#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

#define TEMPERATURE 0
#define PRESSURE 1

// ***********************************************************************
// ************************* BMP085 Subclass *****************************
// ***********************************************************************
class BarometricSensorBMP085 : public BarometricSensor {
// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio
private:
  byte overSamplingSetting;
  int ac1, ac2, ac3;
  unsigned int ac4, ac5, ac6;
  int b1, b2, mb, mc, md;
  long pressure;
  long temperature;
  int altitudeAddress;
  long rawPressure, rawTemperature;
  byte select, pressureCount;
  unsigned long pressureTime;
  unsigned long previousPressureTime;
//  float pressureFactor;
  
  void requestRawPressure(void);
  
  long readRawPressure(void);
  
  void requestRawTemperature(void);
  
  unsigned int readRawTemperature(void);

public: 
  BarometricSensorBMP085();

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void);
  
  void measure(void);
};

#endif