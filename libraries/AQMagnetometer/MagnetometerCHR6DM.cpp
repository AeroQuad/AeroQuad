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

// Class to define sensors that can determine absolute heading


#include "MagnetometerCHR6DM.h"

MagnetometerCHR6DM::MagnetometerCHR6DM(CHR6DM chr6dm)
{
  _chr6dm = &chr6dm;
}

void MagnetometerCHR6DM::initialize(void) {}
  
void MagnetometerCHR6DM::measure(float angleRoll, float anglePitch)
{
  heading = _chr6dm->data.yaw; //this hardly needs any filtering :)
  // Change from +/-180 to 0-360
  if (heading < 0) 
  {
    absoluteHeading = 360 + heading;
  }
  else 
  {
	absoluteHeading = heading;
  }
}
  
const int MagnetometerCHR6DM::getRawData(byte axis)
{
  return absoluteHeading;
}
