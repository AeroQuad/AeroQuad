/*
  AeroQuad v1.7 - March 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

#include "Eeprom.h"

// Utilities for writing and reading from the EEPROM
float readFloat(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;
  
  for (int i = 0; i < 4; i++) 
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void writeFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;
  
  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++) 
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void readEEPROM() {
  PID[ROLL].P = readFloat(PGAIN_ADR);
  PID[ROLL].I = readFloat(IGAIN_ADR);
  PID[ROLL].D = readFloat(DGAIN_ADR);
  PID[ROLL].lastPosition = 0;
  PID[ROLL].integratedError = 0;
  
  PID[PITCH].P = readFloat(PITCH_PGAIN_ADR);
  PID[PITCH].I = readFloat(PITCH_IGAIN_ADR);
  PID[PITCH].D = readFloat(PITCH_DGAIN_ADR);
  PID[PITCH].lastPosition = 0;
  PID[PITCH].integratedError = 0;
  
  PID[YAW].P = readFloat(YAW_PGAIN_ADR);
  PID[YAW].I = readFloat(YAW_IGAIN_ADR);
  PID[YAW].D = readFloat(YAW_DGAIN_ADR);
  PID[YAW].lastPosition = 0;
  PID[YAW].integratedError = 0;
  
  PID[LEVELROLL].P = readFloat(LEVEL_PGAIN_ADR);
  PID[LEVELROLL].I = readFloat(LEVEL_IGAIN_ADR);
  PID[LEVELROLL].D = readFloat(LEVEL_DGAIN_ADR);
  PID[LEVELROLL].lastPosition = 0;
  PID[LEVELROLL].integratedError = 0;  
  
  PID[LEVELPITCH].P = readFloat(LEVEL_PITCH_PGAIN_ADR);
  PID[LEVELPITCH].I = readFloat(LEVEL_PITCH_IGAIN_ADR);
  PID[LEVELPITCH].D = readFloat(LEVEL_PITCH_DGAIN_ADR);
  PID[LEVELPITCH].lastPosition = 0;
  PID[LEVELPITCH].integratedError = 0;
  
  PID[HEADING].P = readFloat(HEADING_PGAIN_ADR);
  PID[HEADING].I = readFloat(HEADING_IGAIN_ADR);
  PID[HEADING].D = readFloat(HEADING_DGAIN_ADR);
  PID[HEADING].lastPosition = 0;
  PID[HEADING].integratedError = 0;
  
  mTransmitter[THROTTLE] = readFloat(THROTTLESCALE_ADR);
  bTransmitter[THROTTLE] = readFloat(THROTTLEOFFSET_ADR);
  mTransmitter[ROLL] = readFloat(ROLLSCALE_ADR);
  bTransmitter[ROLL] = readFloat(ROLLOFFSET_ADR);
  mTransmitter[PITCH] = readFloat(PITCHSCALE_ADR);
  bTransmitter[PITCH] = readFloat(PITCHOFFSET_ADR);
  mTransmitter[YAW] = readFloat(YAWSCALE_ADR);
  bTransmitter[YAW] = readFloat(YAWOFFSET_ADR);
  mTransmitter[MODE] = readFloat(MODESCALE_ADR);
  bTransmitter[MODE] = readFloat(MODEOFFSET_ADR);
  mTransmitter[AUX] = readFloat(AUXSCALE_ADR);
  bTransmitter[AUX] = readFloat(AUXOFFSET_ADR);

  windupGuard = readFloat(WINDUPGUARD_ADR);
  levelLimit = readFloat(LEVELLIMIT_ADR);
  levelOff = readFloat(LEVELOFF_ADR);
  xmitFactor = readFloat(XMITFACTOR_ADR);
  smoothFactor[GYRO] = readFloat(GYROSMOOTH_ADR);
  smoothFactor[ACCEL] = readFloat(ACCSMOOTH_ADR);
  smoothTransmitter[THROTTLE] = readFloat(THROTTLESMOOTH_ADR);
  smoothTransmitter[ROLL] = readFloat(ROLLSMOOTH_ADR);
  smoothTransmitter[PITCH] = readFloat(PITCHSMOOTH_ADR);
  smoothTransmitter[YAW] = readFloat(YAWSMOOTH_ADR);
  smoothTransmitter[MODE] = readFloat(MODESMOOTH_ADR);
  smoothTransmitter[AUX] = readFloat(AUXSMOOTH_ADR);
  accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
  accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
  accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[YAW] = readFloat(GYRO_YAW_ZERO_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
}
