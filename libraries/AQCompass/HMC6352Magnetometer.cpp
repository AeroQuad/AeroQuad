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


#include "HMC6352Magnetometer.h"
#include <Wire.h>


HMC6352Magnetometer::HMC6352Magnetometer()
{
  HMC6352Address = 0x21;
}

// ***********************************************************
// Define all the virtual functions declared in the main class
// ***********************************************************
void HMC6352Magnetometer::initialize()
{
	Wire.beginTransmission(HMC6352Address);
	Wire.send(0x41);		// warm up by making read
	Wire.endTransmission();
}

const int HMC6352Magnetometer::getRawData(byte axis)
{
	return compassTrueHeading;
}

void HMC6352Magnetometer::measure(const float rollAngle, const float pitchAngle)
{
	if((abs(rollAngle)<2) && (abs(pitchAngle)<2)) { // no use reading it unless level
	Wire.requestFrom(HMC6352Address, 2);      // MSB first
	  i = 0;
	  while(Wire.available() && i < 2)
	  {
	    headingData[i] = Wire.receive();
	    i++;
	  }

	  compassTrueHeading = (headingData[0]*256 + headingData[1])/10;

	  Wire.beginTransmission(HMC6352Address); // read to have data available next measure call
	  Wire.send(0x41);
	  Wire.endTransmission();

	  available = true;
	}
	else available = false; // out of angle
}

const int HMC6352Magnetometer::isAvailable()
{
	return available;
}
