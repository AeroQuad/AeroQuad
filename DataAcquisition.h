/*
  AeroQuad v2.1 - January 2011
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

// This header file defines function calls and ISR's needed to communicatw
// over SPI, I2C and other bus communication protocols for measuring sensor data

// ********************************************
// I2C Communication with Wii Sensors
// Original code written by lamarche_mathieu
// Modifications by jihlein 
// ********************************************
// I2C function calls defined in I2C.h

#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    #include <CHR6DM.h>
    CHR6DM chr6dm;

    void initCHR6DM(){
        Serial1.begin(115200); //is this needed here? it's already done in Setup, APM TX1 is closest to board edge, RX1 is one step in (green TX wire from CHR goes into APM RX1)
        chr6dm.resetToFactory();
        chr6dm.setListenMode();
        chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
        chr6dm.requestPacket();
    }

    void readCHR6DM(){
        chr6dm.waitForAndReadPacket();
        chr6dm.requestPacket();
    }
#endif



