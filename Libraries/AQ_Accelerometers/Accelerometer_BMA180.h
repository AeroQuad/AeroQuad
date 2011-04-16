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

#ifndef ACCELEROMETER_BMA180
#define ACCELEROMETER_BMA180

#include "Accelerometer.h"
#include <Wire.h>
#include "..\AQ_I2C\AQ_I2C.h"


class Accelerometer_BMA180 : public Accelerometer {
public:
  I2C i2c;
  Accelerometer_BMA180();
  void initialize(void);
  void measure(void);
  void calibrate(void);	
};

#endif