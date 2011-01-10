/*
  AeroQuad v2.1.3 Beta - December 2010
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

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_PORT.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = SERIAL_PORT.read();
    switch (queryType) {
    case '£':
      SERIAL_PORT.println ((short)NWMP_sx);
      break;
    case 'A': // Receive roll and pitch gyro PID
      readSerialPID(ROLL);
      readSerialPID(PITCH);
      minAcro = readFloatSerial();
      break;
    case 'C': // Receive yaw PID
      readSerialPID(YAW);
      readSerialPID(HEADING);
      headingHoldConfig = readFloatSerial();
      heading = 0;
      relativeHeading = 0;
      headingHold = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      readSerialPID(LEVELROLL);
      readSerialPID(LEVELPITCH);
      readSerialPID(LEVELGYROROLL);
      readSerialPID(LEVELGYROPITCH);
      windupGuard = readFloatSerial();
      break;
    case 'G': // Receive auto level configuration
      levelLimit = readFloatSerial();
      levelOff = readFloatSerial();
      break;
    case 'I': // Receiver altitude hold PID
#ifdef AltitudeHold
      readSerialPID(ALTITUDE);
      PID[ALTITUDE].windupGuard = readFloatSerial();
      minThrottleAdjust = readFloatSerial();
      maxThrottleAdjust = readFloatSerial();
      altitude.setSmoothFactor(readFloatSerial());
      readSerialPID(ZDAMPENING);
#endif
      break;
    case 'K': // Receive data filtering values
      gyro.setSmoothFactor(readFloatSerial());
      accel.setSmoothFactor(readFloatSerial());
      timeConstant = readFloatSerial();
#if defined(AeroQuad_v1) || defined(AeroQuad_v18)
      flightAngle.initialize();
#endif
      break;
    case 'M': // Receive transmitter smoothing values
      receiver.setXmitFactor(readFloatSerial());
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiver.setSmoothFactor(channel, readFloatSerial());
      }
      break;
    case 'O': // Receive transmitter calibration values
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiver.setTransmitterSlope(channel, readFloatSerial());
        receiver.setTransmitterOffset(channel, readFloatSerial());
      }
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
    case 'Y': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      gyro.calibrate();
      accel.calibrate();
      zeroIntegralError();
#ifdef HeadingMagHold
      compass.initialize();
#endif
#ifdef AltitudeHold
      altitude.initialize();
#endif
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
      for (byte motor = FRONT; motor < LASTMOTOR; motor++)
        motors.setRemoteCommand(motor, readFloatSerial());
      break;
    case 'a':
      // spare
      break;
    case 'b': // calibrate gyros
      gyro.calibrate();
      break;
    case 'c': // calibrate accels
      accel.calibrate();
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      flightAngle.calibrate();
      accel.setOneG(accel.getFlightData(ZAXIS));
#endif
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    case 'f': // calibrate magnetometer
#ifdef HeadingMagHold
      compass.setMagCal(XAXIS, readFloatSerial(), readFloatSerial());
      compass.setMagCal(YAXIS, readFloatSerial(), readFloatSerial());
      compass.setMagCal(ZAXIS, readFloatSerial(), readFloatSerial());
#endif
      break;
    case '~': //  read Camera values 
      #ifdef Camera
      camera.setMode(readFloatSerial());
      camera.setCenterPitch(readFloatSerial());
      camera.setCenterRoll(readFloatSerial());
      camera.setCenterYaw(readFloatSerial());
      camera.setmCameraPitch(readFloatSerial());
      camera.setmCameraRoll(readFloatSerial());
      camera.setmCameraYaw(readFloatSerial());
      camera.setServoMinPitch(readFloatSerial());
      camera.setServoMinRoll(readFloatSerial());
      camera.setServoMinYaw(readFloatSerial());
      camera.setServoMaxPitch(readFloatSerial());
      camera.setServoMaxRoll(readFloatSerial());
      camera.setServoMaxYaw(readFloatSerial());
      #endif
      break;
    }
    digitalWrite(LEDPIN, HIGH);
  }
}

//***************************************************************************************************
//********************************* Serial Telemetry ************************************************
//***************************************************************************************************

void PrintValueComma(float val)
{
  SERIAL_PORT.print(val);
  comma();
}

void PrintValueComma(char val)
{
  SERIAL_PORT.print(val);
  comma();
}

void PrintValueComma(int val)
{
  SERIAL_PORT.print(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  SERIAL_PORT.print(val);
  comma();
}

void PrintPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
}

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    //printFreeMemory();
    //SERIAL_PORT.print(gyro.getHeading());
    //comma();
    //SERIAL_PORT.print(batteryMonitor, 2);
    //SERIAL_PORT.println();
    //queryType = 'X';
    break;
  case 'B': // Send roll and pitch gyro PID values
    PrintPID(ROLL);
    PrintPID(PITCH);
    SERIAL_PORT.println(minAcro);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    PrintPID(YAW);
    PrintPID(HEADING);
    SERIAL_PORT.println(headingHoldConfig, BIN);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    PrintPID(LEVELROLL);
    PrintPID(LEVELPITCH);
    PrintPID(LEVELGYROROLL);
    PrintPID(LEVELGYROPITCH);
    SERIAL_PORT.println(windupGuard);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    //SERIAL_PORT.print(levelLimit);
    //comma();
		PrintValueComma(levelLimit);
    SERIAL_PORT.println(levelOff);
    queryType = 'X';
    break;
  case 'J': // Altitude Hold
#ifdef AltitudeHold
    PrintPID(ALTITUDE);
    PrintValueComma(PID[ALTITUDE].windupGuard);
    PrintValueComma(minThrottleAdjust);
    PrintValueComma(maxThrottleAdjust);
    PrintValueComma(altitude.getSmoothFactor());
    PrintValueComma(PID[ZDAMPENING].P);
    PrintValueComma(PID[ZDAMPENING].I);
    SERIAL_PORT.println(PID[ZDAMPENING].D);
#else
    for(byte i=0; i<9; i++) {
      PrintValueComma(0);
    }
    SERIAL_PORT.println('0');
#endif
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    PrintValueComma(gyro.getSmoothFactor());
    PrintValueComma(accel.getSmoothFactor());
    SERIAL_PORT.println(timeConstant);
    // comma();
    // SERIAL_PORT.println(flightMode, DEC);
    queryType = 'X';
    break;
  case 'N': // Send transmitter smoothing values
    PrintValueComma(receiver.getXmitFactor());
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver.getSmoothFactor(axis));
    }
    SERIAL_PORT.println(receiver.getSmoothFactor(AUX));
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver.getTransmitterSlope(axis));
      PrintValueComma(receiver.getTransmitterOffset(axis));
    }
    PrintValueComma(receiver.getTransmitterSlope(AUX));
    SERIAL_PORT.println(receiver.getTransmitterOffset(AUX));
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(gyro.getData(axis));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(accel.getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma(flightAngle.getData(ROLL));
    PrintValueComma(flightAngle.getData(PITCH));
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma(compass.getAbsoluteHeading());
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(altitude.getData());
    #else
      PrintValueComma(0);
    #endif
    #ifdef BattMonitor
      SERIAL_PORT.print(batteryMonitor.getData());
    #else
      SERIAL_PORT.print(0);
    #endif
    SERIAL_PORT.println();
    break;
  case 'R': // Raw magnetometer data
#if defined(HeadingMagHold) && defined(AeroQuad_v2)
    PrintValueComma(compass.getRawData(XAXIS));
    PrintValueComma(compass.getRawData(YAXIS));
    SERIAL_PORT.println(compass.getRawData(ZAXIS));
#else
    PrintValueComma(0);
    PrintValueComma(0);
    SERIAL_PORT.println('0');
#endif
    break;
  case 'S': // Send all flight data
    PrintValueComma(deltaTime);
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(gyro.getFlightData(axis));
    }
    #ifdef BattMonitor
      PrintValueComma(batteryMonitor.getData());
    #else
      PrintValueComma(0);
    #endif
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(motors.getMotorAxisCommand(axis));
    }
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) {
      PrintValueComma(motors.getMotorCommand(motor));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(accel.getFlightData(axis));
    }
    SERIAL_PORT.print(armed, BIN);
    comma();
    if (flightMode == STABLE)
      PrintValueComma(2000);
    if (flightMode == ACRO)
      PrintValueComma(1000);
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma(compass.getAbsoluteHeading());
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(altitude.getData());
      SERIAL_PORT.print(altitudeHold, DEC);
    #else
      PrintValueComma(0);
      SERIAL_PORT.print('0');
    #endif
    SERIAL_PORT.println();    
    break;
  case 'T': // Send processed transmitter values
    PrintValueComma(receiver.getXmitFactor());
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(receiver.getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma(motors.getMotorAxisCommand(ROLL));
    PrintValueComma(motors.getMotorAxisCommand(PITCH));
    SERIAL_PORT.println(motors.getMotorAxisCommand(YAW));
    break;
  case 'U': // Send smoothed receiver with Transmitter Factor applied values
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver.getData(channel));
    }
    SERIAL_PORT.println(receiver.getData(AUX));
    break;
  case 'V': // Send receiver status
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver.getRaw(channel));
    }
    SERIAL_PORT.println(receiver.getRaw(AUX));
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    PrintValueComma(receiver.getData(YAW));
    PrintValueComma(headingHold);
    PrintValueComma(setHeading);
    SERIAL_PORT.println(relativeHeading);
    break;
  case '6': // Report remote commands
    for (byte motor = FRONT; motor < LEFT; motor++) {
      PrintValueComma(motors.getRemoteCommand(motor));
    }
    SERIAL_PORT.println(motors.getRemoteCommand(LEFT));
    break;
  case '!': // Send flight software version
    SERIAL_PORT.println(VERSION, 2);  // jihlein: Print 2 decimal places
    queryType = 'X';
    break;
  case '#': // Send software configuration
    // Determine which hardware is used to define max/min sensor values for Configurator plots
#if defined(AeroQuad_v1)
    PrintValueComma(0);
#elif defined(AeroQuadMega_v1)
    PrintValueComma(1);
#elif defined(AeroQuad_v18)
    PrintValueComma(2);
#elif defined(AeroQuadMega_v2)
    PrintValueComma(3);
#elif defined(AeroQuad_Wii)
    PrintValueComma(4);
#elif defined(AeroQuadMega_Wii)
    PrintValueComma(5);
#elif defined(ArduCopter)
    PrintValueComma(6);
#elif defined(Multipilot)
    PrintValueComma(7);
#elif defined(MultipilotI2C)
    PrintValueComma(8);
#elif defined(AeroQuadMega_CHR6DM)
    PrintValueComma(5);
#elif defined(APM_OP_CHR6DM)
    PrintValueComma(6);
#endif
    // Determine which motor flight configuration for Configurator GUI
#if defined(plusConfig)
    SERIAL_PORT.print('0');
#elif defined(XConfig)
    SERIAL_PORT.print('1');
#elif defined(HEXACOAXIAL)
    SERIAL_PORT.print('2');
#elif defined(HEXARADIAL)
    SERIAL_PORT.print('3');
#endif
    SERIAL_PORT.println();
    queryType = 'X';
    break;  
  case 'e': // Send AREF value
    SERIAL_PORT.println(aref);
    queryType = 'X';
    break;
  case 'g': // Send magnetometer cal values
#ifdef HeadingMagHold
    SERIAL_PORT.print(compass.getMagMax(XAXIS), 2);
    comma();
    SERIAL_PORT.print(compass.getMagMin(XAXIS), 2);
    comma();
    SERIAL_PORT.print(compass.getMagMax(YAXIS), 2);
    comma();
    SERIAL_PORT.print(compass.getMagMin(YAXIS), 2);
    comma();
    SERIAL_PORT.print(compass.getMagMax(ZAXIS), 2);
    comma();
    SERIAL_PORT.println(compass.getMagMin(ZAXIS), 2);
#endif
    queryType = 'X';
    break;
  case '`': // Send Camera values 
    #ifdef Camera
    //SERIAL_PORT.print(camera.getMode());
    //comma();
    PrintValueComma(camera.getMode());
    //SERIAL_PORT.print(camera.getCenterPitch());
    //comma();
    PrintValueComma(camera.getCenterPitch());
    //SERIAL_PORT.print(camera.getCenterRoll());
    //comma();
    PrintValueComma(camera.getCenterRoll());
    //SERIAL_PORT.print(camera.getCenterYaw());
    //comma();
    PrintValueComma(camera.getCenterYaw());
    SERIAL_PORT.print(camera.getmCameraPitch() , 2);
    comma();
    SERIAL_PORT.print(camera.getmCameraRoll() , 2);
    comma();
    SERIAL_PORT.print(camera.getmCameraYaw() , 2);
    comma();
    //SERIAL_PORT.print(camera.getServoMinPitch());
    //comma();
    PrintValueComma(camera.getServoMinPitch());
    //SERIAL_PORT.print(camera.getServoMinRoll());
    //comma();
    PrintValueComma(camera.getServoMinRoll());
    //SERIAL_PORT.print(camera.getServoMinYaw());
    //comma();
    PrintValueComma(camera.getServoMinYaw());
    //SERIAL_PORT.print(camera.getServoMaxPitch());
    //comma();
    PrintValueComma(camera.getServoMaxPitch());
    //SERIAL_PORT.print(camera.getServoMaxRoll());
    //comma();
    PrintValueComma(camera.getServoMaxRoll());
    SERIAL_PORT.println(camera.getServoMaxYaw());
    #endif
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  #define SERIALFLOATSIZE 10
  byte index = 0;
  byte timeout = 0;
  char data[SERIALFLOATSIZE] = "";

  do {
    if (SERIAL_PORT.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SERIAL_PORT.read();
      timeout = 0;
      index++;
    }
  }  
  while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';
  return atof(data);
}

void comma() {
  SERIAL_PORT.print(',');
}

void printInt(int data) {
  byte msb, lsb;

  msb = data >> 8;
  lsb = data & 0xff;

  SERIAL_PORT.print(msb, BYTE);
  SERIAL_PORT.print(lsb, BYTE);
}

