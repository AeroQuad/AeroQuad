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

/*
  When powering up the MaxBotix rangefinders, 
  ensure nothing is within 6" (15cm) of the sensor.
  Otherwise, the readings will be off.
*/
#include <SensorsStatus.h>
#include <MaxSonarRangeFinder.h>

unsigned long timer = 0;

void setup() 
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Rangefinder test library (MaxBotix)");
  
  inititalizeRangeFinders();
}

void loop() 
{
  if ((millis() - timer) > 20) // 50Hz
  {
    timer = millis();
    updateRangeFinders();
    
    Serial.print("Altitude = ");
    Serial.println(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
    //Serial.print(", Front distance = ");
    //Serial.print(rangeFinderRange[FRONT_RANGE_FINDER_INDEX]);
    //Serial.print(", Right distance = ");
    //Serial.print(rangeFinderRange[RIGHT_RANGE_FINDER_INDEX]);
    //Serial.print(", Rear distance = ");
    //Serial.print(rangeFinderRange[REAR_RANGE_FINDER_INDEX]);
    //Serial.print(", Left distance = ");
    //Serial.println(rangeFinderRange[LEFT_RANGE_FINDER_INDEX]);
  }
}
