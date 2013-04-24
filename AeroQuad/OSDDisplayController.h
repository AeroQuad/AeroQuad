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

typedef struct OSDItem {
  short type;
  short row;
  short col;
} OSDItem;

#define OSD_ITEMS_PER_SCREEN 8
typedef OSDItem OSDConfig[OSD_ITEMS_PER_SCREEN];

#define OSD_NUMBER_OF_SCREENS 2
OSDConfig OSDscreen[OSD_NUMBER_OF_SCREENS] = {
  {{1,0,0},{0,0,0},{3,1,1},{0,0,0},{0,0,0},{2,2,1},{0,0,0},{0,0,0}},
  {{0,0,0},{0,0,0},{3,10,10},{0,0,0},{0,0,0},{2,12,12},{0,0,0},{0,0,0}}};

byte OSDsched = 0;
byte OSDlastScreen = 0;
byte OSDcurrentScreen = 0;
unsigned long OSDreinitNeeded = 0xffffffff;

byte foobar=0;

void updateOSD() {
  if (detectVideoStandard(false)) {
    OSDreinitNeeded = 0xffffffff; // Force redraw as display was cleared
    OSDlastScreen = OSDcurrentScreen;
  }
  else if (OSDcurrentScreen != OSDlastScreen) {
    clearOSD();
    OSDreinitNeeded = 0xffffffff;
    OSDlastScreen = OSDcurrentScreen;
  }

  byte firstitem=0, lastitem=0;
  // NOTE: items are updated according to follwing order on 8 rounds
  // item0 rounds 0,2,4,6 // meant for AI
  // item1 rounds 1,5
  // item2,3,4 round 3
  // item5,6,7 round 7
  foobar++;
  OSDcurrentScreen =(foobar>>6);

  // Check notify first, if it did something we dont't have time for other stuff
  if (displayNotify()) {
    return;
  }

  if (!(OSDsched & 1)) { // rounds 0,2,4,6
    firstitem = 0;
    lastitem = 0;
  }
  else if (!(OSDsched & 2)) { // round 1,5
    firstitem = 1;
    lastitem = 1;
  }
  else if (OSDsched == 3) { // round 3
    firstitem = 2;
    lastitem = 4;
  }
  else { // round 7
    firstitem = 5;
    lastitem = 7;
  }

  OSDsched = (OSDsched+1) & 7;

  if ((OSDcurrentScreen >= 0) && (OSDcurrentScreen < OSD_NUMBER_OF_SCREENS)) {
    for ( byte item = firstitem; item < (lastitem + 1); item++) {
      byte  type = OSDscreen[OSDcurrentScreen][item].type;
      if (type) {
        short row = OSDscreen[OSDcurrentScreen][item].row;
        short col = OSDscreen[OSDcurrentScreen][item].col;
        boolean reinit = (OSDreinitNeeded & (1<<type));
	if (reinit) {
	  OSDreinitNeeded &= ~(1 << type);
	}
	switch (type) {
	case 1:
	  {
	    byte extendedFlightMode = flightMode;
#if defined UseGPSNavigator
	    if (ON == positionHoldState) extendedFlightMode = 2;
	    if (ON == navigationState) extendedFlightMode = 3;
#endif
	    displayArtificialHorizon(reinit, kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], extendedFlightMode);
	  }
	  break;
	case 2:
	  displayHeading(row,col,reinit,trueNorthHeading);
	  break;
	case 3:
	  displayRSSI(row,col,reinit);
	  break;
	case 4:
	  displayFlightTime(row,col,reinit,motorArmed);
	  break;
	case 5:
	  displayAltitude(row,col,reinit,getBaroAltitude(), baroAltitudeToHoldTarget, altitudeHoldState);
	  break;
	case 6:
	  //	  displayVariometer(row,col,reinit,climb_fallRate);
	  break;
	case 7:
#ifdef BattMonitor
	  displayVoltage(row,col,reinit,motorArmed);
#endif
	  break;
	case 8:
#ifdef UseGPS
	  displayGPS(row,col,reinit,currentPosition, haveAGpsLock()?missionPositionToReach:currentPosition, haveAGpsLock()?getGpsSpeed():0, haveAGpsLock()?getCourse():0, trueNorthHeading, gpsData.sats);
#endif
	  break;
	case 9:
#ifdef AltitudeHoldRangeFinder
	  if (motorArmed) {
	  displayRanger(col,row,reinit);
	}
#endif
	  break;
	case 10:
	  displayCallSign(row,col,reinit);
	  break;
	case 11:
	  break;
	case 12:
	  break;
	default:
	  break;
	}
      }
    }
  }
}

#endif
