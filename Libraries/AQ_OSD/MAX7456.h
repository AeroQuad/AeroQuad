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

/*
 This module provides on screen display (OSD) for Aeroquad FPV flying.

 It can display
  - artificial horizon with pitch lines
  - battery information
  - altitude (and altitude hold state and target )
  - compass heading
  - flight timer
  - callsign
  - RSSI information
  - additional notification strings

 You will need to upload a special character set initially using the
 provided MAX7456_Font_Updater sketch.

 The user must connect a MAX7456 OSD chip to the appropriate header pins on
 the Arduino. These pins are marked 'OSD' on the AeroQuad Shield v2.

 If the chip is not connected properly, this code should not hang (according
 to my best knowledge).

 If using the SparkFun MAX7456 breakout board, the reset pin should be wired
 high (+5V) either directly or with 10kOhm resistor.

 As the MAX7456 may draw up to 100mA it is a good idea to power it using
 separate regulator (or power it from one of the BEC:s on ESCs). It is known
 that powering from arduino and using >=3S battery will overheat the regulator
 which will lead to Arduino crash.

 Special thanks to Alamo for contributing this capability!

 */

#ifndef _AQ_OSD_MAX7456_H_
#define _AQ_OSD_MAX7456_H_

#include <stdio.h>
#include <stdarg.h>

#include "OSD.h"
#include "GlobalDefined.h"

#include "MAX7456_Config.h" // User configuration

#include "MAX7456_Base.h"   // writeChars, detectVideoStandard, init

#include "MAX7456_Notify.h" // Notification system

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Flight time Display //////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#include "MAX7456_Timer.h"

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// Battery voltage Display //////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef BattMonitor
  #include "MAX7456_BattMonitor.h"
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// AltitudeHold Display /////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
  #include "MAX7456_Altitude.h"
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// HeadingMagHold Display ///////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef HeadingMagHold
  #include "MAX7456_Heading.h"
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)
#ifdef ShowRSSI
  #include "MAX7456_RSSI.h"
#endif

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// ATTITUDE Display /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
#ifdef ShowAttitudeIndicator
  #include "MAX7456_AI.h"
#endif

#endif  // #define _AQ_OSD_MAX7456_H_


