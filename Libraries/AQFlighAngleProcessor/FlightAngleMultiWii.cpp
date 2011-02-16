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

#include "FlightAngleMultiWii.h"
#include <AQMath.h>

FlightAngleMultiWii::FlightAngleMultiWii()
{
  RxEst = 0; // init acc in stable mode
  RyEst = 0;
  RzEst = 1;
  wGyro = 50.0f; // gyro weight/smooting factor
}

void FlightAngleMultiWii::calculate(float G_Dt) 
{
  //get accelerometer readings in g, gives us RAcc vector
  RxAcc = _accel->getRaw(0);
  RyAcc = _accel->getRaw(1);
  RzAcc = _accel->getRaw(2);
  
  //normalize vector (convert to a vector with same direction and with length 1)
  R = sqrt(square(RxAcc) + square(RyAcc) + square(RzAcc));
  RxAcc /= R;
  RyAcc /= R;  
  RzAcc /= R;  
  
  gyroFactor = G_Dt/83e6; //empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
    
  //evaluate R Gyro vector
  if(abs(RzEst) < 0.1f) 
  {
    //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
    //in this case skip the gyro data and just use previous estimate
    RxGyro = RxEst;
    RyGyro = RyEst;
    RzGyro = RzEst;
  }
  else 
  {
    //get angles between projection of R on ZX/ZY plane and Z axis, based on last REst
    //Convert ADC value for to physical units
    //For gyro it will return  deg/ms (rate of rotation)
    atanx = atan2(RxEst,RzEst);
    atany = atan2(RyEst,RzEst);
    
    Axz = atanx + _gyro->getRaw(0)  * gyroFactor;  // convert ADC value for to physical units
    Ayz = atany + _gyro->getRaw(1) * gyroFactor; // and get updated angle according to gyro movement
   
    //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
    signRzGyro = ( cos(Axz) >=0 ) ? 1 : -1;
  
    //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
    RxGyro = sin(Axz) / sqrt( 1 + square(cos(Axz)) * square(tan(Ayz)) );
    RyGyro = sin(Ayz) / sqrt( 1 + square(cos(Ayz)) * square(tan(Axz)) );        
    RzGyro = signRzGyro * sqrt(1 - square(RxGyro) - square(RyGyro));
  }
    
  //combine Accelerometer and gyro readings
  RxEst = (RxAcc + wGyro* RxGyro) / (1.0 + wGyro);
  RyEst = (RyAcc + wGyro* RyGyro) / (1.0 + wGyro);
  RzEst = (RzAcc + wGyro* RzGyro) / (1.0 + wGyro);
  
  _angle[0]  =  180/PI * Axz;
  _angle[1] =  180/PI * Ayz;
}
  
float FlightAngleMultiWii::getGyroUnbias(byte axis) 
{
  return _gyro->getFlightData(axis);
}

 