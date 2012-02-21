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

#ifndef _OSD_DISPLAY_CONTROLLER_H_
#define _OSD_DISPLAY_CONTROLLER_H_

#include "OSD.h"

void updateOSD() {

  // OSD is updated fully in 4 rounds, these are (using bit)
  // 0x01 - Attitude Indicator
  // 0x02 - Altitude, Heading, Timer, RSSI, Reticle
  // 0x04 - Attitude Indicator
  // 0x08 - Battery info

  // Check notify first, if it did something we dont't have time for other stuff
  if (displayNotify()) {
    return;
  }

  #ifdef ShowAttitudeIndicator
    if ((OSDsched&0x01) || (OSDsched&0x04)) {
      displayArtificialHorizon(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], flightMode);
    }
  #endif

  if (OSDsched&0x02) {
    displayFlightTime(motorArmed);
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      displayAltitude(getAltitudeFromSensors(), altitudeToHoldTarget, altitudeHoldState);
    #endif
    #ifdef HeadingMagHold
      displayHeading(kinematicsGetDegreesHeading(ZAXIS));
    #endif
    #ifdef ShowRSSI
      displayRSSI();
    #endif
  }

  if (OSDsched&0x08) {
    #ifdef BattMonitor
      displayVoltage(motorArmed);
    #endif
  }

  OSDsched <<= 1;
  if (OSDsched & 0x10) {
    OSDsched = 0x01;
  }
}

#endif
