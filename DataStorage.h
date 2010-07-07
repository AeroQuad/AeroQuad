/*
  AeroQuad v2.0 - July 2010
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
  
  PID[LEVELGYROROLL].P = readFloat(LEVEL_GYRO_ROLL_PGAIN_ADR);
  PID[LEVELGYROROLL].I = readFloat(LEVEL_GYRO_ROLL_IGAIN_ADR);
  PID[LEVELGYROROLL].D = readFloat(LEVEL_GYRO_ROLL_DGAIN_ADR);
  PID[LEVELGYROROLL].lastPosition = 0;
  PID[LEVELGYROROLL].integratedError = 0;
  
  PID[LEVELGYROPITCH].P = readFloat(LEVEL_GYRO_PITCH_PGAIN_ADR);
  PID[LEVELGYROPITCH].I = readFloat(LEVEL_GYRO_PITCH_IGAIN_ADR);
  PID[LEVELGYROPITCH].D = readFloat(LEVEL_GYRO_PITCH_DGAIN_ADR);
  PID[LEVELGYROPITCH].lastPosition = 0;
  PID[LEVELGYROPITCH].integratedError = 0;

  receiver.setTransmitterSlope(THROTTLE, THROTTLESCALE_ADR);
  receiver.setTransmitterOffset(THROTTLE, THROTTLEOFFSET_ADR);
  receiver.setTransmitterSlope(ROLL, ROLLSCALE_ADR);
  receiver.setTransmitterOffset(ROLL, ROLLOFFSET_ADR);
  receiver.setTransmitterSlope(PITCH, PITCHSCALE_ADR);
  receiver.setTransmitterOffset(PITCH, PITCHOFFSET_ADR);
  receiver.setTransmitterSlope(YAW, YAWSCALE_ADR);
  receiver.setTransmitterOffset(YAW, YAWOFFSET_ADR);
  receiver.setTransmitterSlope(MODE, MODESCALE_ADR);
  receiver.setTransmitterOffset(MODE, MODEOFFSET_ADR);
  receiver.setTransmitterSlope(AUX, AUXSCALE_ADR);
  receiver.setTransmitterOffset(AUX, AUXOFFSET_ADR);

  windupGuard = readFloat(WINDUPGUARD_ADR);
  levelLimit = readFloat(LEVELLIMIT_ADR);
  levelOff = readFloat(LEVELOFF_ADR);
  receiver.setXmitFactor(XMITFACTOR_ADR);
  //smoothFactor[GYRO] = readFloat(GYROSMOOTH_ADR); // Now loaded in gyro class
  //smoothFactor[ACCEL] = readFloat(ACCSMOOTH_ADR); //  Now loaded in accel class
  //smoothTransmitter[THROTTLE] = readFloat(THROTTLESMOOTH_ADR); // Now loaded in receiver class
  //smoothTransmitter[ROLL] = readFloat(ROLLSMOOTH_ADR); // Now loaded in receiver class
  //smoothTransmitter[PITCH] = readFloat(PITCHSMOOTH_ADR); // Now loaded in receiver class
  //smoothTransmitter[YAW] = readFloat(YAWSMOOTH_ADR); // Now loaded in receiver class
  //smoothTransmitter[MODE] = readFloat(MODESMOOTH_ADR); // Now loaded in receiver class
  //smoothTransmitter[AUX] = readFloat(AUXSMOOTH_ADR); // Now loaded in receiver class
  //accel.setZero(ROLL, readFloat(LEVELROLLCAL_ADR));     // Now loaded in accel class
  //accel.setZero(PITCH, readFloat(LEVELPITCHCAL_ADR));   // Now loaded in accel class
  //accel.setZero(ZAXIS, readFloat(LEVELZCAL_ADR));       // Now loaded in accel class
  //gyro.setZero(ROLL, readFloat(GYRO_ROLL_ZERO_ADR));    // Now loaded in gyro class
  //gyro.setZero(PITCH, readFloat(GYRO_PITCH_ZERO_ADR));  // Now loaded in gyro class
  //gyro.setZero(YAW, readFloat(GYRO_YAW_ZERO_ADR));      // Now loaded in gyro class
  timeConstant = readFloat(FILTERTERM_ADR);
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);
  minAcro = readFloat(MINACRO_ADR);
}
