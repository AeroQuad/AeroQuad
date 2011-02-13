/*
  AeroQuad v2.2 - Feburary 2011
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

// Written by Lokling & Honk: http://aeroquad.com/showthread.php?1287-Experimental-CHR6DM-sensor-board

// Usage: define a global var such as  "CHR6DM chr6 ;" in Aeroquad.pde
// Values can then be read such as chr6.data.pitch and so on

#ifndef _AQ_CHR6DM_SENSORS_ACCESSOR_H_
#define _AQ_CHR6DM_SENSORS_ACCESSOR_H_

#include "WProgram.h"

struct Data
{
  boolean yawEnabled;
  boolean pitchEnabled;
  boolean rollEnabled;
  boolean yawRateEnabled;
  boolean pitchRateEnabled;
  boolean rollRateEnabled;
  boolean mxEnabled;
  boolean myEnabled;
  boolean mzEnabled;
  boolean gxEnabled;
  boolean gyEnabled;
  boolean gzEnabled;
  boolean axEnabled;
  boolean ayEnabled;
  boolean azEnabled;

  double yaw;
  double pitch;
  double roll;
  double yawRate;
  double pitchRate;
  double rollRate;
  double mx;
  double my;
  double mz;
  double gx;
  double gy;
  double gz;
  double ax;
  double ay;
  double az;
};


class CHR6DM 
{
public:

  Data data;
  
  float CHR_RollAngle;
  float CHR_PitchAngle;

  CHR6DM();

  void readCHR6DM();
  void EKFReset();
  void writeToFlash();
  int readPacket();
  int blockingRead();
  boolean syncToHeader();
  void resetToFactory();
  bool setActiveChannels(int channels);
  void setBroadCastMode(int x);
  void sendPacket(int command);
  void sendPacket(int command, int* bytes, int byteslength);
  boolean requestPacket();
  boolean waitForAndReadPacket();
  bool requestAndReadPacket();
  bool waitFor(int command,int timeout);
  boolean decodePacket();
  boolean selfTest();
  int bytesToSignedShort(int high, int low);
  boolean setListenMode();
  boolean waitForAck(int timeout);
  void initCHR6DM();
};


#endif // #define _AQ_CHR6DM_SENSORS_H_