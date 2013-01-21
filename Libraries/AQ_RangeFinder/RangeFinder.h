/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AEROQUAD_RANGE_FINDER_H_
#define _AEROQUAD_RANGE_FINDER_H_

#include "Arduino.h"

#define INVALID_RANGE -1
#define MISSING_RANGE -2

#define ALTITUDE_RANGE_FINDER_INDEX 0
#define FRONT_RANGE_FINDER_INDEX    1
#define RIGHT_RANGE_FINDER_INDEX    2
#define REAR_RANGE_FINDER_INDEX     3
#define LEFT_RANGE_FINDER_INDEX     4

float rangeFinderRange[5]      = {-2,-2,-2,-2,-2};

float maxRangeFinderRange = 4.5;
float minRangeFinderRange = 0.0;


void    inititalizeRangeFinders();
void    updateRangeFinders();

boolean isOnRangerRange(float distance) {

  return (distance >= minRangeFinderRange) && (distance <= maxRangeFinderRange);
}

boolean isRangerPresent(byte idx) {
  
  return (rangeFinderRange[idx] > -2);
}

#endif //  #ifdef _AEROQUAD_RANGE_FINDER_H_
