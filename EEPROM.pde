/*
  AeroQuad v1.2 - August 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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
  accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
  accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
  accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[YAW] = readFloat(GYRO_YAW_ZERO_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
  
  // Check if certain key values are not populated
  // Put default value in if it is zero
  if (PID[ROLL].P == 0) PID[ROLL].P = 3.0;
  if (PID[PITCH].P == 0) PID[PITCH].P = 3.0;
  if (PID[YAW].P == 0) PID[YAW].P = 10.0;
  if (smoothFactor[GYRO] == 0) smoothFactor[GYRO] = 0.20;
  if (smoothFactor[ACCEL] == 0) smoothFactor[ACCEL] = 0.20;
  if (timeConstant == 0) timeConstant = 5.0;  
  if (windupGuard == 0) windupGuard = 2000.0;
  if (xmitFactor == 0) xmitFactor = 0.20;  
  if (levelLimit == 0) levelLimit = 2000.0;
  if (levelOff == 0) levelOff = 50;  
  if (smoothTransmitter[THROTTLE] == 0) smoothTransmitter[THROTTLE] = 1.0;
  if (smoothTransmitter[ROLL] == 0) smoothTransmitter[ROLL] = 1.0;
  if (smoothTransmitter[PITCH] == 0) smoothTransmitter[PITCH] = 1.0;
  if (smoothTransmitter[YAW] == 0) smoothTransmitter[YAW] = 1.0;  
  if (accelZero[ROLL] == 0) {
    zeroGyros();
    zeroAccelerometers();
    zeroIntegralError();
  }
}
