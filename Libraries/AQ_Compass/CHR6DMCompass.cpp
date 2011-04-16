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

#include "CHR6DMCompass.h"

void CHR6DMCompass::measure(const float rollAngle, const float pitchAngle) 
{
  _heading = _chr6dm->data.yaw; //this hardly needs any filtering :)
  // Change from +/-180 to 0-360
  if (_heading < 0)
  {
    _absoluteHeading = 360 + _heading;
  }
  else 
  {
    _absoluteHeading = _heading;
  }
}
  
void CHR6DMCompass::setChr6dm(CHR6DM *chr6dm)
{
  _chr6dm = chr6dm;
}
