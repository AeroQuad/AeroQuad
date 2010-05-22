/*
  AeroQuad v1.8 - May 2010
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

void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = Serial.read();
    switch (queryType) {
    case 'A': // Receive roll and pitch gyro PID
      PID[ROLL].P = readFloatSerial();
      PID[ROLL].I = readFloatSerial();
      PID[ROLL].D = readFloatSerial();
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH].P = readFloatSerial();
      PID[PITCH].I = readFloatSerial();
      PID[PITCH].D = readFloatSerial();
      PID[PITCH].lastPosition = 0;
      PID[PITCH].integratedError = 0;
      break;
    case 'C': // Receive yaw PID
      PID[YAW].P = readFloatSerial();
      PID[YAW].I = readFloatSerial();
      PID[YAW].D = readFloatSerial();
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      PID[HEADING].P = readFloatSerial();
      PID[HEADING].I = readFloatSerial();
      PID[HEADING].D = readFloatSerial();
      PID[HEADING].lastPosition = 0;
      PID[HEADING].integratedError = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      PID[LEVELROLL].P = readFloatSerial();
      PID[LEVELROLL].I = readFloatSerial();
      PID[LEVELROLL].D = readFloatSerial();
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH].P = readFloatSerial();
      PID[LEVELPITCH].I = readFloatSerial();
      PID[LEVELPITCH].D = readFloatSerial();
      PID[LEVELPITCH].lastPosition = 0;
      PID[LEVELPITCH].integratedError = 0;
      PID[LEVELGYROROLL].P = readFloatSerial();
      PID[LEVELGYROROLL].I = readFloatSerial();
      PID[LEVELGYROROLL].D = readFloatSerial();
      PID[LEVELGYROROLL].lastPosition = 0;
      PID[LEVELGYROROLL].integratedError = 0;
      PID[LEVELGYROPITCH].P = readFloatSerial();
      PID[LEVELGYROPITCH].I = readFloatSerial();
      PID[LEVELGYROPITCH].D = readFloatSerial();
      PID[LEVELGYROPITCH].lastPosition = 0;
      PID[LEVELGYROPITCH].integratedError = 0;
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial();
      levelOff = readFloatSerial();
      break;
    case 'I': // Receive flight control configuration
      windupGuard = readFloatSerial();
      xmitFactor = readFloatSerial();
      break;
    case 'K': // Receive data filtering values
      smoothFactor[GYRO] = readFloatSerial();
      smoothFactor[ACCEL] = readFloatSerial();
      timeConstant = readFloatSerial();
      break;
    case 'M': // Receive transmitter smoothing values
      smoothTransmitter[ROLL] = readFloatSerial();
      smoothTransmitter[PITCH] = readFloatSerial();
      smoothTransmitter[YAW] = readFloatSerial();
      smoothTransmitter[THROTTLE] = readFloatSerial();
      smoothTransmitter[MODE] = readFloatSerial();
      smoothTransmitter[AUX] = readFloatSerial();
      break;
    case 'O': // Receive transmitter calibration values
      mTransmitter[ROLL] = readFloatSerial();
      bTransmitter[ROLL] = readFloatSerial();
      mTransmitter[PITCH] = readFloatSerial();
      bTransmitter[PITCH] = readFloatSerial();
      mTransmitter[YAW] = readFloatSerial();
      bTransmitter[YAW] = readFloatSerial();
      mTransmitter[THROTTLE] = readFloatSerial();
      bTransmitter[THROTTLE] = readFloatSerial();
      mTransmitter[MODE] = readFloatSerial();
      bTransmitter[MODE] = readFloatSerial();
      mTransmitter[AUX] = readFloatSerial();
      bTransmitter[AUX] = readFloatSerial();
      break;
    case 'W': // Write all user configurable values to EEPROM
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
      writeFloat(xmitFactor, XMITFACTOR_ADR);
      writeFloat(smoothFactor[GYRO], GYROSMOOTH_ADR);
      writeFloat(smoothFactor[ACCEL], ACCSMOOTH_ADR);
      writeFloat(smoothTransmitter[THROTTLE], THROTTLESMOOTH_ADR);
      writeFloat(smoothTransmitter[ROLL], ROLLSMOOTH_ADR);
      writeFloat(smoothTransmitter[PITCH], PITCHSMOOTH_ADR);
      writeFloat(smoothTransmitter[YAW], YAWSMOOTH_ADR);
      writeFloat(smoothTransmitter[MODE], MODESMOOTH_ADR);
      writeFloat(smoothTransmitter[AUX], AUXSMOOTH_ADR);
      writeFloat(timeConstant, FILTERTERM_ADR);
      writeFloat(mTransmitter[THROTTLE], THROTTLESCALE_ADR);
      writeFloat(bTransmitter[THROTTLE], THROTTLEOFFSET_ADR);
      writeFloat(mTransmitter[ROLL], ROLLSCALE_ADR);
      writeFloat(bTransmitter[ROLL], ROLLOFFSET_ADR);
      writeFloat(mTransmitter[PITCH], PITCHSCALE_ADR);
      writeFloat(bTransmitter[PITCH], PITCHOFFSET_ADR);
      writeFloat(mTransmitter[YAW], YAWSCALE_ADR);
      writeFloat(bTransmitter[YAW], YAWOFFSET_ADR);
      writeFloat(mTransmitter[MODE], MODESCALE_ADR);
      writeFloat(bTransmitter[MODE], MODEOFFSET_ADR);
      writeFloat(mTransmitter[AUX], AUXSCALE_ADR);
      writeFloat(bTransmitter[AUX], AUXOFFSET_ADR);
      writeFloat(smoothHeading, HEADINGSMOOTH_ADR);
      writeFloat(aref, AREF_ADR);
      writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
      zeroIntegralError();
      // Complementary filter setup
      for (axis = ROLL; axis < YAW; axis++)
        angle[axis].initialize(axis); // defined in FlightAngle.h
      break;
    case 'Y': // Initialize EEPROM with default values
      PID[ROLL].P = 5;
      PID[ROLL].I = 0;
      PID[ROLL].D = -10;
      PID[PITCH].P = 5;
      PID[PITCH].I = 0;
      PID[PITCH].D = -10;
      PID[YAW].P = 12.0;
      PID[YAW].I = 0;
      PID[YAW].D = 0;
      PID[LEVELROLL].P = 12;
      PID[LEVELROLL].I = 0;
      PID[LEVELROLL].D = -1;
      PID[LEVELPITCH].P = 12;
      PID[LEVELPITCH].I = 0;
      PID[LEVELPITCH].D = -1;
      PID[HEADING].P = 3;
      PID[HEADING].I = 0;
      PID[HEADING].D = 0;
      PID[LEVELGYROROLL].P = 7;
      PID[LEVELGYROROLL].I = 0;
      PID[LEVELGYROROLL].D = -15;
      PID[LEVELGYROPITCH].P = 7;
      PID[LEVELGYROPITCH].I = 0;
      PID[LEVELGYROPITCH].D = -15;
      windupGuard = 2000.0;
      xmitFactor = 0.20;  
      levelLimit = 2000.0;
      levelOff = 50;  
      smoothFactor[GYRO] = 0.50;
      smoothFactor[ACCEL] = 0.50;
      timeConstant = 4.0;   
      for (channel = ROLL; channel < LASTCHANNEL; channel++) {
        mTransmitter[channel] = 1.0;
        bTransmitter[channel] = 0.0;
      }
      smoothTransmitter[THROTTLE] = 1.0;
      smoothTransmitter[ROLL] = 1.0;
      smoothTransmitter[PITCH] = 1.0;
      smoothTransmitter[YAW] = 0.5;
      smoothTransmitter[MODE] = 1.0;
      smoothTransmitter[AUX] = 1.0;
      smoothHeading = 1.0;

      autoZeroGyros();
      zeroGyros();
      zeroAccelerometers();
      aref = 3.0; // Use 2.8 if you are using an AeroQuad Shield < v1.7
      zeroIntegralError();
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      armed = 0;
      calibrateESC = 1;
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
      armed = 0;
      calibrateESC = 2;
      break;
    case '3': // Test ESC calibration
      armed = 0;
      testCommand = readFloatSerial();
      calibrateESC = 3;
      break;
    case '4': // Turn off ESC calibration
      armed = 0;
      calibrateESC = 0;
      testCommand = 1000;
      break;
    case '5': // Send individual motor commands (motor, command)
      armed = 0;
      calibrateESC = 5;
      for (motor = FRONT; motor < LASTMOTOR; motor++)
        remoteCommand[motor] = readFloatSerial();
      break;
    case 'a': // Enable/disable fast data transfer of sensor data
      queryType = 'X'; // Stop any other telemetry streaming
      if (readFloatSerial() == 1)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    case 'b': // calibrate gyros
      autoZeroGyros();
      zeroGyros();
      break;
    case 'c': // calibrate accels
      zeroAccelerometers();
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    }
  digitalWrite(LEDPIN, HIGH);
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}
