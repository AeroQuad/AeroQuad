/*
  AeroQuad v2.2 - Feburary 2011
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

#ifndef _AQ_BMP085_BAROMETRIC_SENSOR_H_
#define _AQ_BMP085_BAROMETRIC_SENSOR_H_

#include "AltitudeProvider.h"

class BMP085BarometricSensor : public AltitudeProvider 
{
// This sets up the BMP085 from Sparkfun
// Code from http://wiring.org.co/learning/libraries/bmp085.html
// Also made bug fixes based on BMP085 library from Jordi Munoz and Jose Julio
private:
  byte _select;
  byte _pressureCount;
  byte _overSamplingSetting;
  int _ac1;
  int _ac2;
  int _ac3;
  int _b1;
  int _b2;
  int _mc;
  int _md;
  int _altitudeAddress;
  unsigned int _ac4;
  unsigned int _ac5;
  unsigned int _ac6;
  long _pressure;
  long _temperature;
  long _rawPressure;
  long _rawTemperature;
  float _pressureFactor;
  
  void requestRawPressure();
  long readRawPressure();
  void requestRawTemperature(); 
  unsigned int readRawTemperature();

public: 
  BMP085BarometricSensor();

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize();
  void measure();
};

#endif