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

#ifndef _AQ_FLIGHT_ANGLE_PROCESSOR_WII_FILTER_H_
#define _AQ_FLIGHT_ANGLE_PROCESSOR_WII_FILTER_H_

#include "FlightAngleProcessor.h"

// ***********************************************************************
// ********************* MultiWii Kalman Filter **************************
// ***********************************************************************
// Original code by Alex at: http://radio-commande.com/international/triwiicopter-design/
// ************************************
// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// with this algorithm, we can get absolute angles for a stable mode integration
// ************************************

class FlightAngleMultiWii : public FlightAngleProcessor 
{ 
private:
  int8_t signRzGyro;  
  float R;
  float RxEst; // init acc in stable mode
  float RyEst;
  float RzEst;
  float Axz,Ayz;           //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
  float RxAcc,RyAcc,RzAcc;         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer       
  float RxGyro,RyGyro,RzGyro;        //R obtained from last estimated value and gyro movement
  float wGyro; // gyro weight/smooting factor
  float atanx,atany;
  float gyroFactor;
  //float meanTime; // **** Need to update this ***

public: 
  FlightAngleMultiWii();

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {}
  
  void calculate(float G_Dt);
  
  float getGyroUnbias(byte axis);

  void calibrate(void) {}
};

#endif