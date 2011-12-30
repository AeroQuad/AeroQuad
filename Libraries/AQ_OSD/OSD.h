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

#ifndef _AQ_OSD_H_
#define _AQ_OSD_H_

byte OSDsched = 0;

#ifdef BattMonitor
  void displayVoltage(byte areMotorsArmed);
#endif
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  void displayAltitude(float readedAltitude, float desiredAltitudeToKeep, boolean altitudeHoldState);
#endif
#ifdef HeadingMagHold
  void displayHeading(int currentHeading);
#endif
#ifdef ShowFlightTimer
  void displayFlightTime(byte areMotorsArmed);
#endif
#ifdef ShowRSSI
  void displayRSSI();
#endif
#ifdef ShowAttitudeIndicator
  void displayArtificialHorizon(float roll, float pitch);
#endif
#ifdef ShowReticle
  void displayReticle(byte flightMode);
#endif

void initializeOSD();
void updateOSD();
byte displayNotify();
byte notifyOSDmenu(byte flags, byte cursorLeft, byte cursorRight, const char *fmt, ...);

#endif