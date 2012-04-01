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

#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_H_

// @see http://www.arduino.cc/playground/Main/MaxSonar

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)


#include "RangeFinder.h"

#define ALTITUDE_RANGE_FINDER_PIN 1 // analog
#define FRONT_RANGE_FINDER_PIN    5	// analog
#define RIGHT_RANGE_FINDER_PIN    4	// analog
#define REAR_RANGE_FINDER_PIN     3	// analog
#define LEFT_RANGE_FINDER_PIN     2	// analog




byte rangeFinderPins[5] = {ALTITUDE_RANGE_FINDER_PIN,
						   FRONT_RANGE_FINDER_PIN,
						   RIGHT_RANGE_FINDER_PIN,
						   REAR_RANGE_FINDER_PIN,
						   LEFT_RANGE_FINDER_PIN};

//
// default unit are centimeter
//

// default min max range constrain

void inititalizeRangeFinder(byte idx) {

  maxRangeFinderRange = 4.5;
  minRangeFinderRange = 0.25;
  vehicleState |= RANGE_ENABLED;
  
  pinMode(rangeFinderPins[idx], INPUT);
}

boolean isInRangeOfRangeFinder(float range) {
  return ((range <= maxRangeFinderRange) && (range >= minRangeFinderRange));
}

void readRangeFinder(byte idx) {
  rangeFinderRange[idx] = (analogRead(rangeFinderPins[idx]) * 1.8333) / 100.0;
  if (!isInRangeOfRangeFinder(rangeFinderRange[idx])) {
    rangeFinderRange[idx] = INVALID_RANGE;
  }
}



#endif 

#endif








