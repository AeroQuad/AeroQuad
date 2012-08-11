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

#ifndef _AQ_EzUHF_RSSI_READER_H_
#define _AQ_EzUHF_RSSI_READER_H_

#include <Receiver.h>

#define RSSI_RAWVAL         // show raw A/D value instead of percents (for tuning)
#define RSSI_MAX_RAW_VALUE 1001
#define RSSI_MIN_RAW_VALUE 2000
#define SIGNAL_QUALITY_MAX_RAW_VALUE 2001
#define SIGNAL_QUALITY_MIN_RAW_VALUE 1614

#define RSSI_WARN    20     // show alarm at %

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)

short rssiRawValue = 0; //forces update at first run
short signalQualityRawValue = 0;

void readRSSI() {

  rssiRawValue = map(receiverCommand[AUX2],RSSI_MIN_RAW_VALUE,RSSI_MAX_RAW_VALUE,0,100);
  rssiRawValue = constrain(rssiRawValue,0,100);
  signalQualityRawValue = map(receiverCommand[AUX3],SIGNAL_QUALITY_MIN_RAW_VALUE,SIGNAL_QUALITY_MAX_RAW_VALUE,0,100);
  signalQualityRawValue = constrain(signalQualityRawValue,0,100);
}

#endif  // #define _AQ_OSD_MAX7456_RSSI_H_
