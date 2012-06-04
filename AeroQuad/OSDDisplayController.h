/*
 AeroQuad v3.0 - December 2011
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

#ifndef _OSD_DISPLAY_CONTROLLER_H_
#define _OSD_DISPLAY_CONTROLLER_H_

#include "OSD.h"

byte OSDsched = 0;

void updateOSD() {
  // OSD is updated fully in 8 rounds
  // 1,3,5,7 - Attitude Indicator - updated at 5Hz
  // 2       - Altitude, Heading, Timer, RSSI - updated at 1.25Hz
  // 4       - Battery info
  // 6,8     - GPS (internally 2 phases: Position & Navigation

  // Check notify first, if it did something we dont't have time for other stuff
  if (displayNotify()) {
    return;
  }

  #ifdef ShowAttitudeIndicator
    if (OSDsched&0x55) {
      byte extendedFlightMode = flightMode;
      #if defined UseGPSNavigator
        if (ON == positionHoldState) extendedFlightMode = 2;
        if (ON == navigationState) extendedFlightMode = 3;
      #endif
      displayArtificialHorizon(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], extendedFlightMode);
    }
  #endif

  if (OSDsched&0x02) {
    displayFlightTime(motorArmed);
    #if defined AltitudeHoldBaro
      displayAltitude(getBaroAltitude(), baroAltitudeToHoldTarget, altitudeHoldState);
    #endif
    #ifdef HeadingMagHold
      displayHeading(trueNorthHeading);
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

  if (OSDsched&0x20) {
    #ifdef UseGPS
      if (haveAGpsLock()) {
        displayGPS(currentPosition, missionPositionToReach, getGpsSpeed(), getCourse(), trueNorthHeading, nbSatelitesInUse);
      }
      else {
        displayGPS(currentPosition, currentPosition, 0, 0, trueNorthHeading, nbSatelitesInUse);
      }
    #endif
  }

  if (OSDsched&0x80) {
    #ifdef AltitudeHoldRangeFinder
      if (motorArmed) {
        displayRanger();
      }
    #endif
  }

  OSDsched <<= 1;
  if (!OSDsched) {
    OSDsched = 0x01;
  }
}

#endif
