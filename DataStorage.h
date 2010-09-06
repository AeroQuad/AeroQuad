/*
  AeroQuad v2.0 - September 2010
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

// contains all default values when re-writing EEPROM
void initializeEEPROM(void) {
  PID[ROLL].P = 1.0;
  PID[ROLL].I = 0.0;
  PID[ROLL].D = -7.0;
  PID[PITCH].P = 1.0;
  PID[PITCH].I = 0.0;
  PID[PITCH].D = -7.0;
  PID[YAW].P = 3.0;
  PID[YAW].I = 0.0;
  PID[YAW].D = 0.0;
  PID[LEVELROLL].P = 15.0;
  PID[LEVELROLL].I = 0.0;
  PID[LEVELROLL].D = 0.0;
  PID[LEVELPITCH].P = 15;
  PID[LEVELPITCH].I = 0.0;
  PID[LEVELPITCH].D = 0.0;
  PID[HEADING].P = 3;
  PID[HEADING].I = 0;
  PID[HEADING].D = 0;
  PID[LEVELGYROROLL].P = 0.6;
  PID[LEVELGYROROLL].I = 0.0;
  PID[LEVELGYROROLL].D = -15;
  PID[LEVELGYROPITCH].P = 0.6;
  PID[LEVELGYROPITCH].I = 0.0;
  PID[LEVELGYROPITCH].D = -15;
  windupGuard = 2000.0;
  receiver.setXmitFactor(0.60);  
  levelLimit = 90.0;
  levelOff = 0.0;
  gyro.setSmoothFactor(1.0);
  accel.setSmoothFactor(1.0);
  timeConstant = 7.0;   
  for (channel = ROLL; channel < LASTCHANNEL; channel++) {
    receiver.setTransmitterSlope(channel, 1.0);
    receiver.setTransmitterOffset(channel, 0.0);
  }
  receiver.setSmoothFactor(THROTTLE, 1.0);
  receiver.setSmoothFactor(ROLL, 1.0);
  receiver.setSmoothFactor(PITCH, 1.0);
  receiver.setSmoothFactor(YAW, 0.5);
  receiver.setSmoothFactor(MODE, 1.0);
  receiver.setSmoothFactor(AUX, 1.0);
  smoothHeading = 1.0;
  flightMode = ACRO;
  headingHoldConfig = OFF;
  minAcro = 1300;
  aref = 3.0; // Use 2.8 if you are using an AeroQuad Shield < v1.7
}

void readEEPROM(void) {
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

void writeEEPROM(void){
  writeFloat(PID[ROLL].P, PGAIN_ADR);
  writeFloat(PID[ROLL].I, IGAIN_ADR);
  writeFloat(PID[ROLL].D, DGAIN_ADR);
  writeFloat(PID[PITCH].P, PITCH_PGAIN_ADR);
  writeFloat(PID[PITCH].I, PITCH_IGAIN_ADR);
  writeFloat(PID[PITCH].D, PITCH_DGAIN_ADR);
  writeFloat(PID[LEVELROLL].P, LEVEL_PGAIN_ADR);
  writeFloat(PID[LEVELROLL].I, LEVEL_IGAIN_ADR);
  writeFloat(PID[LEVELROLL].D, LEVEL_DGAIN_ADR);
  writeFloat(PID[LEVELPITCH].P, LEVEL_PITCH_PGAIN_ADR);
  writeFloat(PID[LEVELPITCH].I, LEVEL_PITCH_IGAIN_ADR);
  writeFloat(PID[LEVELPITCH].D, LEVEL_PITCH_DGAIN_ADR);
  writeFloat(PID[YAW].P, YAW_PGAIN_ADR);
  writeFloat(PID[YAW].I, YAW_IGAIN_ADR);
  writeFloat(PID[YAW].D, YAW_DGAIN_ADR);
  writeFloat(PID[HEADING].P, HEADING_PGAIN_ADR);
  writeFloat(PID[HEADING].I, HEADING_IGAIN_ADR);
  writeFloat(PID[HEADING].D, HEADING_DGAIN_ADR);
  writeFloat(PID[LEVELGYROROLL].P, LEVEL_GYRO_ROLL_PGAIN_ADR);
  writeFloat(PID[LEVELGYROROLL].I, LEVEL_GYRO_ROLL_IGAIN_ADR);
  writeFloat(PID[LEVELGYROROLL].D, LEVEL_GYRO_ROLL_DGAIN_ADR);
  writeFloat(PID[LEVELGYROPITCH].P, LEVEL_GYRO_PITCH_PGAIN_ADR);
  writeFloat(PID[LEVELGYROPITCH].I, LEVEL_GYRO_PITCH_IGAIN_ADR);
  writeFloat(PID[LEVELGYROPITCH].D, LEVEL_GYRO_PITCH_DGAIN_ADR);
  writeFloat(windupGuard, WINDUPGUARD_ADR);  
  writeFloat(levelLimit, LEVELLIMIT_ADR);   
  writeFloat(levelOff, LEVELOFF_ADR); 
  writeFloat(receiver.getXmitFactor(), XMITFACTOR_ADR);
  writeFloat(gyro.getSmoothFactor(), GYROSMOOTH_ADR);
  writeFloat(accel.getSmoothFactor(), ACCSMOOTH_ADR);
  writeFloat(receiver.getSmoothFactor(THROTTLE), THROTTLESMOOTH_ADR);
  writeFloat(receiver.getSmoothFactor(ROLL), ROLLSMOOTH_ADR);
  writeFloat(receiver.getSmoothFactor(PITCH), PITCHSMOOTH_ADR);
  writeFloat(receiver.getSmoothFactor(YAW), YAWSMOOTH_ADR);
  writeFloat(receiver.getSmoothFactor(MODE), MODESMOOTH_ADR);
  writeFloat(receiver.getSmoothFactor(AUX), AUXSMOOTH_ADR);
  writeFloat(timeConstant, FILTERTERM_ADR);
  writeFloat(receiver.getTransmitterSlope(THROTTLE), THROTTLESCALE_ADR);
  writeFloat(receiver.getTransmitterOffset(THROTTLE), THROTTLEOFFSET_ADR);
  writeFloat(receiver.getTransmitterSlope(ROLL), ROLLSCALE_ADR);
  writeFloat(receiver.getTransmitterOffset(ROLL), ROLLOFFSET_ADR);
  writeFloat(receiver.getTransmitterSlope(PITCH), PITCHSCALE_ADR);
  writeFloat(receiver.getTransmitterOffset(PITCH), PITCHOFFSET_ADR);
  writeFloat(receiver.getTransmitterSlope(YAW), YAWSCALE_ADR);
  writeFloat(receiver.getTransmitterOffset(YAW), YAWOFFSET_ADR);
  writeFloat(receiver.getTransmitterSlope(MODE), MODESCALE_ADR);
  writeFloat(receiver.getTransmitterOffset(MODE), MODEOFFSET_ADR);
  writeFloat(receiver.getTransmitterSlope(AUX), AUXSCALE_ADR);
  writeFloat(receiver.getTransmitterOffset(AUX), AUXOFFSET_ADR);
  writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
  writeFloat(aref, AREF_ADR);
  writeFloat(flightMode, FLIGHTMODE_ADR);
  writeFloat(headingHoldConfig, HEADINGHOLD_ADR);
  writeFloat(minAcro, MINACRO_ADR);
}
