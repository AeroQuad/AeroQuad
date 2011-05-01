/*
  AeroQuad v3.0 - May 2011
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

#ifndef _AEROQUAD_ACCELEROMETER_BMA180_H_
#define _AEROQUAD_ACCELEROMETER_BMA180_H_

#include <Accelerometer.h>

#define ACCEL_ADDRESS 0x40
#define ACCEL_IDENTITY 0x03
#define ACCEL_RESET_REGISTER 0x10
#define ACCEL_TRIGER_RESET_VALUE 0xB6
#define ACCEL_ENABLE_WRITE_CONTROL_REGISTER 0x0D
#define ACCEL_CONTROL_REGISTER 0x10
#define ACCEL_BW_TCS 0x20
#define ACCEL_LOW_PASS_FILTER_REGISTER 0x20
#define ACCEL_10HZ_LOW_PASS_FILTER_VALUE 0x0F
#define ACCEL_OFFSET_REGISTER 0x35
#define ACCEL_READ_ROLL_ADDRESS 0x02
#define ACCEL_READ_PITCH_ADDRESS 0x04
#define ACCEL_READ_YAW_ADDRESS 0x06

class Accelerometer_BMA180 : public Accelerometer {
public:
  Accelerometer_BMA180();
  
  void initialize(void);
  void measure(void);
  void calibrate(void);
};
#endif
