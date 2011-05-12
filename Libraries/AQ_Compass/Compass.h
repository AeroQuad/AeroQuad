/*
  AeroQuad v3.0 - April 2011
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


#ifndef _AEROQUAD_COMPASS_H_
#define _AEROQUAD_COMPASS_H_

#include <WProgram.h>

class Compass {
private:
  float magMax[3];
  float magMin[3];
  
protected:  
  float hdgX;
  float hdgY;

  float measuredMagX;
  float measuredMagY;
  float measuredMagZ;
  float magScale[3];
  float magOffset[3];
  float magFieldBodyRaw[3];

public:

  Compass();
  
  virtual void initialize(float dcm[]) {}
  virtual void measure(float roll, float pitch, float dcm[], byte useMagBias) {}
  
  const float getHdgXY(byte axis);
  const int getRawData(byte axis);
  void setMagCal(byte axis, float maxValue, float minValue);
  const float getMagMax(byte axis);
  const float getMagMin(byte axis);
  
  
	
};



#endif