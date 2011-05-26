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
#include <WProgram.h>
#include "UP501.h"

  void UP501::initialize()
  {
    Serial1.begin(9600);
    Serial1.println("$PMTK251,38400*27");

    Serial1.end();

    delay(100);

    Serial1.begin(38400);

    Serial1.println("$PMTK300,200,0,0,0,0*2F");
  }

  void UP501::measure()  
  {
    while (Serial1.available())
    {
      byte b = Serial1.read();
      if (gps.encode(b))
      {
        gps.get_position(&latitude, &longitude, &position_age);
		altitude = gps.altitude();
		speed = gps.speed();
        return;
      }
      //Serial.write(b);
    }
  }

