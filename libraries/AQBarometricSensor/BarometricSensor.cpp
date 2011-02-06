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

#include "BarometricSensor.h"
#include "WProgram.h"


BarometricSensor::BarometricSensor (void) 
{ 
  altitude = 0;
  smoothFactor = 0.02;
}

// **********************************************************************
// The following function calls must be defined inside any new subclasses
// **********************************************************************
void BarometricSensor::initialize(void) {} 
void BarometricSensor::measure(void) {}
  
// *********************************************************
// The following functions are common between all subclasses
// *********************************************************
const float BarometricSensor::getData(void) 
{
  return altitude - groundAltitude;
}
  
const float BarometricSensor::getRawData(void) 
{
  return rawAltitude;
}
  
void BarometricSensor::setStartAltitude(float value) 
{
  altitude = value;
}
  
void BarometricSensor::measureGround(void) 
{
  // measure initial ground pressure (multiple samples)
  groundAltitude = 0;
  for (int i=0; i < 25; i++) 
  {
    measure();
    delay(26);
    groundAltitude += rawAltitude;
  }
  groundAltitude = groundAltitude / 25.0;
}
  
void BarometricSensor::setGroundAltitude(float value) 
{
  groundAltitude = value;
}
  
const float BarometricSensor::getGroundAltitude(void) 
{
  return groundAltitude;
}
  
void BarometricSensor::setSmoothFactor(float value) 
{
  smoothFactor = value;
}
  
const float BarometricSensor::getSmoothFactor(void) 
{
  return smoothFactor;
}
