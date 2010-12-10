/*
  AeroQuad v2.1 Beta - December 2010
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

// Special thanks for 1k space optimization update from Ala42
// http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code&p=13359&viewfull=1#post13359

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

void readPID(unsigned char IDPid, unsigned char IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  pid->P = readFloat(IDEeprom);
  pid->I = readFloat(IDEeprom+4);
  pid->D = readFloat(IDEeprom+8);
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void writePID(unsigned char IDPid, unsigned int IDEeprom) {
  struct PIDdata* pid = &PID[IDPid];
  writeFloat(pid->P, IDEeprom);
  writeFloat(pid->I, IDEeprom+4);
  writeFloat(pid->D, IDEeprom+8);
}

// contains all default values when re-writing EEPROM
void initializeEEPROM(void) {
  PID[ROLL].P = 1.20;
  PID[ROLL].I = 0.0;
  PID[ROLL].D = -0.05;
  PID[PITCH].P = 1.20;
  PID[PITCH].I = 0.0;
  PID[PITCH].D = -0.05;
  PID[YAW].P = 3.0;
  PID[YAW].I = 0.05;
  PID[YAW].D = 0.0;
  PID[LEVELROLL].P = 7.0;
  PID[LEVELROLL].I = 1.0;
  PID[LEVELROLL].D = 0.0;
  PID[LEVELPITCH].P = 7.0;
  PID[LEVELPITCH].I = 1.0;
  PID[LEVELPITCH].D = 0.0;
  PID[HEADING].P = 3.0;
  PID[HEADING].I = 0.1;
  PID[HEADING].D = 0.0;
  PID[LEVELGYROROLL].P = 1.2;
  PID[LEVELGYROROLL].I = 0.0;
  PID[LEVELGYROROLL].D = -0.05;
  PID[LEVELGYROPITCH].P = 1.2;
  PID[LEVELGYROPITCH].I = 0.0;
  PID[LEVELGYROPITCH].D = -0.05;
  #ifdef AltitudeHold
    PID[ALTITUDE].P = 25.0;
    PID[ALTITUDE].I = 0.1;
    PID[ALTITUDE].D = 0.0;
    PID[ALTITUDE].windupGuard = 25.0; //this prevents the 0.1 I term to rise too far
    PID[ZDAMPENING].P = 0.0;
    PID[ZDAMPENING].I = 0.0;
    PID[ZDAMPENING].D = 0.0;
    minThrottleAdjust = -50.0;
    maxThrottleAdjust = 50.0; //we don't want it to be able to take over totally
    altitude.setSmoothFactor(0.1);
  #endif
  #ifdef HeadingMagHold
    compass.setMagCal(XAXIS, 1, 0);
    compass.setMagCal(YAXIS, 1, 0);
    compass.setMagCal(ZAXIS, 1, 0);
  #endif
  windupGuard = 1000.0;
  receiver.setXmitFactor(0.20);  //Honk
  levelLimit = 500.0;
  levelOff = 150.0;
  gyro.setSmoothFactor(1.0);
  accel.setSmoothFactor(1.0);
  accel.setOneG(500);
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
  aref = 5.0; // Use 3.0 if using a v1.7 shield or use 2.8 for an AeroQuad Shield < v1.7
}

void readEEPROM(void) {
  readPID(ROLL, ROLL_PID_GAIN_ADR);
  readPID(PITCH, PITCH_PID_GAIN_ADR);
  readPID(YAW, YAW_PID_GAIN_ADR);
  readPID(LEVELROLL, LEVELROLL_PID_GAIN_ADR);
  readPID(LEVELPITCH, LEVELPITCH_PID_GAIN_ADR);
  readPID(HEADING, HEADING_PID_GAIN_ADR);
  readPID(LEVELGYROROLL, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  readPID(LEVELGYROPITCH, LEVEL_GYRO_PITCH_PID_GAIN_ADR);
  PID[LEVELGYROPITCH].lastPosition = 0;
  PID[LEVELGYROPITCH].integratedError = 0;
  
  #ifdef AltitudeHold
    //readPID(ALTITUDE, ALTITUDE_PGAIN_ADR);
    PID[ALTITUDE].P = readFloat(ALTITUDE_PGAIN_ADR);
    PID[ALTITUDE].I = readFloat(ALTITUDE_IGAIN_ADR);
    PID[ALTITUDE].D = readFloat(ALTITUDE_DGAIN_ADR);
    PID[ALTITUDE].windupGuard = readFloat(ALTITUDE_WINDUP_ADR);
    PID[ALTITUDE].lastPosition = 0;
    PID[ALTITUDE].integratedError = 0;
    //readPID(ZDAMPENING, ZDAMP_PGAIN_ADR);
    PID[ZDAMPENING].P = readFloat(ZDAMP_PGAIN_ADR);
    PID[ZDAMPENING].I = readFloat(ZDAMP_IGAIN_ADR);
    PID[ZDAMPENING].D = readFloat(ZDAMP_DGAIN_ADR);
    PID[ZDAMPENING].lastPosition = 0;
    PID[ZDAMPENING].integratedError = 0;
    minThrottleAdjust = readFloat(ALTITUDE_MIN_THROTTLE_ADR);
    maxThrottleAdjust = readFloat(ALTITUDE_MAX_THROTTLE_ADR);
    altitude.setSmoothFactor(readFloat(ALTITUDE_SMOOTH_ADR));
  #endif

  #ifdef HeadingMagHold
    compass.setMagCal(XAXIS, readFloat(MAGXMAX_ADR), readFloat(MAGXMIN_ADR));
    compass.setMagCal(YAXIS, readFloat(MAGYMAX_ADR), readFloat(MAGYMIN_ADR));
    compass.setMagCal(ZAXIS, readFloat(MAGZMAX_ADR), readFloat(MAGZMIN_ADR));
  #endif

  windupGuard = readFloat(WINDUPGUARD_ADR);
  levelLimit = readFloat(LEVELLIMIT_ADR);
  levelOff = readFloat(LEVELOFF_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
  smoothHeading = readFloat(HEADINGSMOOTH_ADR);
  aref = readFloat(AREF_ADR);
  flightMode = readFloat(FLIGHTMODE_ADR);
  headingHoldConfig = readFloat(HEADINGHOLD_ADR);
  minAcro = readFloat(MINACRO_ADR);
  accel.setOneG(readFloat(ACCEL1G_ADR));
}

void writeEEPROM(void){
  cli(); // Needed so that APM sensor data doesn't overflow
  writePID(ROLL, ROLL_PID_GAIN_ADR);
  writePID(PITCH, PITCH_PID_GAIN_ADR);
  writePID(LEVELROLL, LEVELROLL_PID_GAIN_ADR);
  writePID(LEVELPITCH, LEVELPITCH_PID_GAIN_ADR);
  writePID(YAW, YAW_PID_GAIN_ADR);
  writePID(HEADING, HEADING_PID_GAIN_ADR);
  writePID(LEVELGYROROLL, LEVEL_GYRO_ROLL_PID_GAIN_ADR);
  writePID(LEVELGYROPITCH, LEVEL_GYRO_PITCH_PID_GAIN_ADR);
  #ifdef AltitudeHold
    writeFloat(PID[ALTITUDE].P, ALTITUDE_PGAIN_ADR);
    writeFloat(PID[ALTITUDE].I, ALTITUDE_IGAIN_ADR);
    writeFloat(PID[ALTITUDE].D, ALTITUDE_DGAIN_ADR);
    writeFloat(PID[ALTITUDE].windupGuard, ALTITUDE_WINDUP_ADR);
    writeFloat(PID[ZDAMPENING].P, ZDAMP_PGAIN_ADR);
    writeFloat(PID[ZDAMPENING].I, ZDAMP_IGAIN_ADR);
    writeFloat(PID[ZDAMPENING].D, ZDAMP_DGAIN_ADR);
    writeFloat(minThrottleAdjust, ALTITUDE_MIN_THROTTLE_ADR);
    writeFloat(maxThrottleAdjust, ALTITUDE_MAX_THROTTLE_ADR);
    writeFloat(altitude.getSmoothFactor(), ALTITUDE_SMOOTH_ADR); 
  #endif
  #ifdef HeadingMagHold
    writeFloat(compass.getMagMax(XAXIS), MAGXMAX_ADR);
    writeFloat(compass.getMagMin(XAXIS), MAGXMIN_ADR);
    writeFloat(compass.getMagMax(YAXIS), MAGYMAX_ADR);
    writeFloat(compass.getMagMin(YAXIS), MAGYMIN_ADR);
    writeFloat(compass.getMagMax(ZAXIS), MAGZMAX_ADR);
    writeFloat(compass.getMagMin(ZAXIS), MAGZMIN_ADR);
  #endif
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
  writeFloat(accel.getOneG(), ACCEL1G_ADR);
  sei(); // Restart interrupts
}
