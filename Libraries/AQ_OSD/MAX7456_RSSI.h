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

#ifndef _AQ_OSD_MAX7456_RSSI_H_
#define _AQ_OSD_MAX7456_RSSI_H_

//////////////////////////////////////////////////////////////////////////////
/////////////////////////// RSSI Display /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// Show RSSI information (analog input value optionally mapped to percents.)
#if defined (UseEzUHFRSSIReader)
  #include <EzUHFRSSIReader.h>
#else
  #include <AnalogRSSIReader.h>
#endif	

short lastRSSI = 1234; //forces update at first run
#if defined (UseEzUHFRSSIReader)
  short lastQuality = 1234;  //forces update at first run
#endif

void displayRSSI() {

  if (rssiRawValue != lastRSSI) {
    lastRSSI = rssiRawValue;
    char buf[6];
    #ifdef RSSI_RAWVAL
      snprintf(buf, 6, "\372%4u", rssiRawValue);
      writeChars(buf, 5, 0, RSSI_ROW, RSSI_COL);
    #else
      snprintf(buf, 6, "\372%3u%%", rssiRawValue);
      writeChars(buf, 5, (RSSI_WARN>rssiRawValue)?1:0, RSSI_ROW, RSSI_COL);
    #endif
  }
  #if defined (UseEzUHFRSSIReader)
    if (lastQuality != signalQualityRawValue) {
	  char buf[6];
	  snprintf(buf, 6, "\372%4u", signalQualityRawValue);
      writeChars(buf, 5, 0, SIGNAL_QUALITY_ROW, SIGNAL_QUALITY_COL);
	  signalQualityRawValue = lastQuality;
	}
  #endif
  
}

#endif  // #define _AQ_OSD_MAX7456_RSSI_H_
