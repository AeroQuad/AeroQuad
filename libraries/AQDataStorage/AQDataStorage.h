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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

// Utilities for writing and reading from the EEPROM

#ifndef _DATA_STORAGE_H_
#define _DATA_STORAGE_H_

#include "WProgram.h"
#include <EEPROM.h>

union FloatStore 
{
  byte floatByte[4];
  float floatVal;
};

float readFloat(int address);

void writeFloat(float value, int address);

#endif