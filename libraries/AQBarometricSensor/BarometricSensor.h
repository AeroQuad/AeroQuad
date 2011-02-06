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

#ifndef _BAROMETRIC_SENSOR_H
#define _BAROMETRIC_SENSOR_H

class BarometricSensor 
{
public:
  double altitude, rawAltitude;
  float groundTemperature; // remove later
  float groundPressure; // remove later
  float groundAltitude;
  float smoothFactor;
  
  BarometricSensor (void);

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void measure(void);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  const float getData(void);
  
  const float getRawData(void);
  
  void setStartAltitude(float value);
  
  void measureGround(void);
  
  void setGroundAltitude(float value);
  
  const float getGroundAltitude(void);
  
  void setSmoothFactor(float value);
  
  const float getSmoothFactor(void);
};

#endif