/*
  AeroQuad v3.0 - March 2011
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

#define ADC_NUMBER_OF_BITS 10

#include <SensorsStatus.h>
#include <MaxSonarRangeFinder.h>

void setup() {
  Serial.begin(115200);
  inititalizeRangeFinders();
}

void loop() {
  
  updateRangeFinders();
  
  Serial.print("Distance = ");
  Serial.println(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
}
