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

#ifndef _AQ_SENSORS_STATE_H_
#define _AQ_SENSORS_STATE_H_

unsigned long vehicleState = 0;

#define GYRO_DETECTED         0x001
#define ACCEL_DETECTED        0x002
#define MAG_DETECTED          0x004
#define BARO_DETECTED         0x008
#define HEADINGHOLD_ENABLED   0x010
#define ALTITUDEHOLD_ENABLED  0x020
#define BATTMONITOR_ENABLED   0x040
#define CAMERASTABLE_ENABLED  0x080
#define RANGE_ENABLED         0x100

#endif