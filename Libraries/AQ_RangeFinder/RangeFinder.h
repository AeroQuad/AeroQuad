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

#ifndef _AEROQUAD_RANGE_FINDER_H_
#define _AEROQUAD_RANGE_FINDER_H_

#include "Arduino.h"
#include "GlobalDefined.h"

#define ALTITUDE_RANGE_FINDER_INDEX 0
#define FRONT_RANGE_FINDER_INDEX    1
#define RIGHT_RANGE_FINDER_INDEX    2
#define REAR_RANGE_FINDER_INDEX     3
#define LEFT_RANGE_FINDER_INDEX     4


							
byte rangeFinderSampleCount[5] = {0,0,0,0,0};							
int rangeFinderRangeSum[5]     = {0,0,0,0,0};								  
float rangeFinderRange[5]      = {0.0,0.0,0.0,0.0,0.0};


//
// default unit are centimeter
//

// default min max range constrain
float maxRangeFinderRange = 3.0;
float minRangeFinderRange = 0.25;

void inititalizeRangeFinder(byte idx);
void readRangeFinderDistanceSum(byte idx);
void evaluateDistanceFromSample(byte idx);
boolean isInRangeOfRangeFinder(byte idx);

#endif //  #ifdef _AEROQUAD_RANGE_FINDER_H_