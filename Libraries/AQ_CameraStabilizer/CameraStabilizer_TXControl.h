/*
 AeroQuad v3.0.1 - February 2012
 www.AeroQuad.com
 Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AEROQUAD_CAMERA_STABILIZER_TXCONTROL_H_
#define _AEROQUAD_CAMERA_STABILIZER_TXCONTROL_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined (AeroQuadSTM32)

#include "CameraStabilizer.h"

// Written by wooden

int servoCenterPitchTemp = 0;
int lastServoCenterPitchTemp = 0;
int servoActualCenter = 1367;		// CALIBRATE SERVOS BEFORE TURNING TX CONTROL ON, SET THIS TO servoPitchCenter FOUND DURING CALIBRATION

//void processCameraTXControl();

void processCameraTXControl() {
  if (LASTCHANNEL >= 8) {
    if (receiverCommand[AUX3] == 1500) { // pot is centered, don't change pitch
      servoCenterPitch = servoActualCenter;
    } else {
      servoCenterPitchTemp = map(receiverCommand[AUX3],1000,2000,servoMaxPitch,servoMinPitch);
      if (servoCenterPitchTemp != lastServoCenterPitchTemp)  { // servo doesn't match channel reading
        servoCenterPitch = servoCenterPitchTemp;
        lastServoCenterPitchTemp = servoCenterPitchTemp;
      }
    }
  }
}
#endif
#endif