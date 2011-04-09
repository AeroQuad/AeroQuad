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

#include "Kinematics.h"
#include <Axis.h>

 
Kinematics::Kinematics(void) {
  for (byte axis = ROLL; axis <= YAW; axis++)
    angle[axis] = 0.0;
  gyroAngle[ROLL] = 0;
  gyroAngle[PITCH] = 0;
}
  
const float Kinematics::getData(byte axis) {
  return angle[axis];
}

const float Kinematics::getHeading(byte axis) {
  return(angle[axis]);
}
  
const float Kinematics::getDegreesHeading(byte axis) {
    
  float tDegrees = degrees(angle[axis]);
  if (tDegrees < 0.0)
    return (tDegrees + 360.0);
  else
    return (tDegrees);
}
  
const byte Kinematics::getType(void) {
  return type;
}
