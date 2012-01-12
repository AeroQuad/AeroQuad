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

#ifndef _AQ_OSD_MAX7456_TIMER_H_
#define _AQ_OSD_MAX7456_TIMER_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Flight time Display //////////////////////////////
//////////////////////////////////////////////////////////////////////////////

unsigned long prevTime = 0;           // previous time since start when OSD.update() ran
unsigned int prevArmedTimeSecs = 111; // bogus to force update
unsigned long armedTime = 0;          // time motors have spent armed

void displayFlightTime(byte areMotorsArmed) {
  if (areMotorsArmed == ON) {
    armedTime += ( currentTime-prevTime );
  }

  prevTime = currentTime;
  unsigned int armedTimeSecs = armedTime / 1000000;
  if (armedTimeSecs != prevArmedTimeSecs) {
    prevArmedTimeSecs = armedTimeSecs;
    char buf[7];
    snprintf(buf,7,"\5%02u:%02u",armedTimeSecs/60,armedTimeSecs%60);
    writeChars(buf, 6, 0, TIMER_ROW, TIMER_COL );
  }
}
#endif  // #define _AQ_OSD_MAX7456_TIMER_H_


