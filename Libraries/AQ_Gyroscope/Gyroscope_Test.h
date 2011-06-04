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

#ifndef _AEROQUAD_GYROSCOPE_TEST_H_
#define _AEROQUAD_GYROSCOPE_TEST_H_

#include <Gyroscope.h>

class Gyroscope_Test : public Gyroscope {
private:
  int gyroADC[3]; // raw data from sensor

public:
  Gyroscope_Test() {
    // Add any required variable initialization here
    scaleFactor = 1.0; // Define the scale factor that converts to radians/second
  }

  void initialize() {
    // Add hardware initialization or setup here
    calibrate(); // Calibrate gyros after each power up, store zero values to EEPROM outside class
  }

  void measure() {
    // Replace code below with sensor measurement methodology
    for (byte axis = 0; axis <3; axis++)
      gyroADC[axis] = random(0, 1024) ;
  
    // Invert axis as needed here by switching gyroADC[] and zero[]
    // Axis definitions: roll right >0, pitch up >0, yaw right >0
    rate[0] = (gyroADC[0] - zero[0]) * scaleFactor;
    rate[1] = (gyroADC[1] - zero[1]) * scaleFactor;
    rate[2] = (gyroADC[2] - zero[2]) * scaleFactor;
  }

  void calibrate() {
    // Add calibration method for measurement when gyro is motionless
    for (byte axis = 0; axis < 3; axis++)
      zero[axis] = random(510, 514); // simulate zero measurement around 512
  }

};
#endif