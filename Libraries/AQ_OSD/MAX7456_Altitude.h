/*
  AeroQuad v3.0 - Nov 2011
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

#ifndef _AQ_OSD_MAX7456_ALTITUDE_H_
#define _AQ_OSD_MAX7456_ALTITUDE_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// AltitudeHold Display /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

int lastAltitude     = 12345;     // bogus initial values to force update
int lastHoldAltitude = 12345;
byte lastHoldState   = 6;

void displayAltitude(float readedAltitude, float desiredAltitudeToKeep, boolean altHoldState) {
  #ifdef feet
    int currentAltitude = readedAltitude*3.281;
    int currentHoldAltitude = desiredAltitudeToKeep*3.281;
  #else // metric
    int currentAltitude = readedAltitude*10.0; // 0.1m accuracy!!
    int currentHoldAltitude = desiredAltitudeToKeep*10.0;
  #endif
  char buf[7];

  if ( lastAltitude != currentAltitude ) {
    #ifdef feet
      snprintf(buf,7,"\10%4df",currentAltitude);
    #else
      if (abs(currentAltitude)<100) {
        snprintf(buf,7,"\010%c%1d.%1dm",currentAltitude < 0 ? '-' : ' ', abs(currentAltitude/10),abs(currentAltitude%10));
      }
      else {
        snprintf(buf,7,"\010%4dm",currentAltitude/10);
      }
    #endif
    writeChars( buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL );
    lastAltitude = currentAltitude;
  }

  // AltitudeHold handling:
  // - show hold altitude when it is active
  // - show "panic" if 'paniced' out
  boolean isWriteNeeded = false;
  switch (altHoldState) {
  case OFF:
    if (lastHoldState != OFF) {
      lastHoldState = OFF;
      memset(buf,0,6);
      isWriteNeeded = true;
    }
    break;
  case ON:
    if ((lastHoldState != ON) || (lastHoldAltitude != currentHoldAltitude)) {
      lastHoldState = ON;
      lastHoldAltitude=currentHoldAltitude;
      #ifdef feet
        snprintf(buf,7,"\11%4df",currentHoldAltitude);
      #else
        if (abs(currentHoldAltitude)<100) {
          snprintf(buf,7,"\011%c%1d.%1dm", currentHoldAltitude < 0 ? '-' : ' ',abs(currentHoldAltitude/10),abs(currentHoldAltitude%10));
        }
        else {
          snprintf(buf,7,"\011%4dm",currentHoldAltitude/10);
        }
      #endif
      isWriteNeeded = true;
    }
    break;
  case ALTPANIC:
    if (lastHoldState != ALTPANIC) {
      lastHoldState = ALTPANIC;
      snprintf(buf,7,"\11panic");
      isWriteNeeded = true;
    }
    break;
  }

  if (isWriteNeeded) {
    writeChars( buf, 6, 0, ALTITUDE_ROW, ALTITUDE_COL+6 );
  }
}

#endif  // #define _AQ_OSD_MAX7456_ALTITUDE_H_


