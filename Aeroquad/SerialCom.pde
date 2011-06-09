/*
  AeroQuad v3.0 - May 2011
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
  if (PIDid == HEADING)
    pid->typePID = TYPEPI;
  else
    pid->typePID = NOTYPE;
  pid->firstPass = true;
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    digitalWrite(LEDPIN, LOW);
    queryType = SERIAL_READ();
    switch (queryType) {
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
      windupGuard = readFloatSerial(); // defaults found in setup() of AeroQuad.pde
      break;
    case 'G': // Receive auto level configuration
      readFloatSerial();
      readFloatSerial();
      break;
    case 'I': // Receiver altitude hold PID
#ifdef AltitudeHold
      readSerialPID(ALTITUDE);
      PID[ALTITUDE].windupGuard = readFloatSerial();
      minThrottleAdjust = readFloatSerial();
      maxThrottleAdjust = readFloatSerial();
      barometricSensor->setSmoothFactor(readFloatSerial());
      readSerialPID(ZDAMPENING);
#endif
      break;
    case 'K': // Receive data filtering values
      gyro->setSmoothFactor(readFloatSerial());
      accel->setSmoothFactor(readFloatSerial());
      timeConstant = readFloatSerial();
      break;
    case 'M': // Receive transmitter smoothing values
      receiver->setXmitFactor(readFloatSerial());
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiver->setSmoothFactor(channel, readFloatSerial());
      }
      break;
    case 'O': // Receive transmitter calibration values
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) {
        receiver->setTransmitterSlope(channel, readFloatSerial());
        receiver->setTransmitterOffset(channel, readFloatSerial());
      }
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
    case 'Y': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      gyro->calibrate();
      accel->calibrate();
      zeroIntegralError();
#ifdef HeadingMagHold
      compass->initialize();
#endif
#ifdef AltitudeHold
      barometricSensor->initialize();
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
      for (byte motor = 0; motor < LASTMOTOR; motor++)
        motorConfiguratorCommand[motor] = (int)readFloatSerial();
      break;
    case 'a': // fast telemetry transfer
      if (readFloatSerial() == 1.0)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    case 'b': // calibrate gyros
      gyro->calibrate();
      storeSensorsZeroToEEPROM();
      break;
    case 'c': // calibrate accels
      accel->calibrate();
      storeSensorsZeroToEEPROM();
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      flightAngle->calibrate();
      accel->setOneG(accel->getOneG(ZAXIS));
#endif
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    case 'f': // calibrate magnetometer
#ifdef HeadingMagHold
      compass->setMagCal(XAXIS, readFloatSerial(), readFloatSerial());
      compass->setMagCal(YAXIS, readFloatSerial(), readFloatSerial());
      compass->setMagCal(ZAXIS, readFloatSerial(), readFloatSerial());
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

void PrintValueComma(float val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(double val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(char val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(int val) {
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  SERIAL_PRINT(val);
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
    //PrintValueComma(gyro->getFlightData(PITCH));
    //PrintValueComma(flightAngle->getData(PITCH));
    //PrintValueComma(flightAngle->getGyroUnbias(PITCH));
    //PrintValueComma(receiver->getZero(ROLL));
    //PrintValueComma(flightAngle->getData(ROLL));
    //Serial.print(degrees(flightAngle->getData(YAW)));
    //Serial.println();
    //printFreeMemory();
    //queryType = 'X';
    break;
  case 'B': // Send roll and pitch gyro PID values
    PrintPID(ROLL);
    PrintPID(PITCH);
    SERIAL_PRINTLN(minAcro);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    PrintPID(YAW);
    PrintPID(HEADING);
    SERIAL_PRINTLN(headingHoldConfig, BIN);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    PrintPID(LEVELROLL);
    PrintPID(LEVELPITCH);
    PrintPID(LEVELGYROROLL);
    PrintPID(LEVELGYROPITCH);
    SERIAL_PRINTLN(windupGuard);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    PrintValueComma(0);
    Serial.println(0);
    queryType = 'X';
    break;
  case 'J': // Altitude Hold
#ifdef AltitudeHold
    PrintPID(ALTITUDE);
    PrintValueComma(PID[ALTITUDE].windupGuard);
    PrintValueComma(minThrottleAdjust);
    PrintValueComma(maxThrottleAdjust);
    PrintValueComma(barometricSensor->getSmoothFactor());
    PrintValueComma(PID[ZDAMPENING].P);
    PrintValueComma(PID[ZDAMPENING].I);
    SERIAL_PRINTLN(PID[ZDAMPENING].D);
#else
    for(byte i=0; i<9; i++) {
     PrintValueComma(0);
    }
    SERIAL_PRINTLN('0');
#endif
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    PrintValueComma(gyro->getSmoothFactor());
    PrintValueComma(accel->getSmoothFactor());
    SERIAL_PRINTLN(timeConstant);
    // comma();
    // Serial.println(flightMode, DEC);
    queryType = 'X';
    break;
  case 'N': // Send transmitter smoothing values
    PrintValueComma(receiver->getXmitFactor());
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver->getSmoothFactor(axis));
    }
    SERIAL_PRINTLN(receiver->getSmoothFactor(AUX));
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver->getTransmitterSlope(axis));
      PrintValueComma(receiver->getTransmitterOffset(axis));
    }
    PrintValueComma(receiver->getTransmitterSlope(AUX));
    SERIAL_PRINTLN(receiver->getTransmitterOffset(AUX));
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(gyro->getRadPerSec(axis));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(accel->getMeterPerSec(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma(degrees(kinematics->getData(ROLL)));
    PrintValueComma(degrees(kinematics->getData(PITCH)));
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      //PrintValueComma(compass->getAbsoluteHeading());
      PrintValueComma(kinematics->getDegreesHeading(YAW));
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(barometricSensor->getAltitude());
    #else
      PrintValueComma(0);
    #endif
    #ifdef BattMonitor
      SERIAL_PRINT(batteryMonitor->getBatteryVoltage());
    #else
      SERIAL_PRINT(0);
    #endif
    SERIAL_PRINTLN();
    break;
  case 'R': // Raw magnetometer data
#if defined(HeadingMagHold)
    PrintValueComma(compass->getRawData(XAXIS));
    PrintValueComma(compass->getRawData(YAXIS));
    SERIAL_PRINTLN(compass->getRawData(ZAXIS));
#else
    PrintValueComma(0);
    PrintValueComma(0);
    SERIAL_PRINTLN('0');
#endif
    break;
  case 'S': // Send all flight data
    PrintValueComma(deltaTime);
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == PITCH)
        PrintValueComma(-gyro->getRadPerSec(axis));
      else
        PrintValueComma(gyro->getRadPerSec(axis));
    }
    #ifdef BattMonitor
      PrintValueComma(batteryMonitor->getBatteryVoltage());
    #else
      PrintValueComma(0);
    #endif
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(motors->getMotorCommand(axis));
    }
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motors->getMotorCommand(motor));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      if (axis == ROLL)
        PrintValueComma(accel->getMeterPerSec(YAXIS));
      else if (axis == PITCH)
        PrintValueComma(accel->getMeterPerSec(XAXIS));
      else
        PrintValueComma(accel->getMeterPerSec(ZAXIS));
    }  
    SERIAL_PRINT(armed, BIN);
    comma();
    if (flightMode == STABLE)
      PrintValueComma(2000);
    if (flightMode == ACRO)
      PrintValueComma(1000);
    #ifdef HeadingMagHold
      //PrintValueComma(compass->getAbsoluteHeading());
      PrintValueComma(kinematics->getDegreesHeading(YAW));
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(barometricSensor->getAltitude());
      SERIAL_PRINT(altitudeHold, DEC);
    #else
      PrintValueComma(0);
      SERIAL_PRINT('0');
    #endif
    SERIAL_PRINTLN();    
    break;
  case 'T': // Send processed transmitter values
    PrintValueComma(receiver->getXmitFactor());
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(receiver->getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma(motorAxisCommandRoll);
    PrintValueComma(motorAxisCommandPitch);
    SERIAL_PRINTLN(motorAxisCommandYaw);
    break;
  case 'U': // Send smoothed receiver with Transmitter Factor applied values
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver->getData(channel));
    }
    SERIAL_PRINTLN(receiver->getData(AUX));
    break;
  case 'V': // Send receiver status
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver->getData(channel));
    }
    SERIAL_PRINTLN(receiver->getData(AUX));
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    PrintValueComma(receiver->getData(YAW));
    PrintValueComma(headingHold);
    PrintValueComma(setHeading);
    // AKA - Configurator wants -180/180 for headings,
    // when heading hold active, the relative heading can be > 180 due to the way it's calculated
    // this corrects it just for the configurator.
    if ((setHeading + relativeHeading) > 180)
      SERIAL_PRINTLN(-360 + relativeHeading);
    else
      SERIAL_PRINTLN(relativeHeading);
    break;
  case '6': // Report remote commands
    for (byte motor = 0; motor < 3; motor++) {
      PrintValueComma(motors->getMotorCommand(motor));
    }
    SERIAL_PRINTLN(motors->getMotorCommand(3));
    break;
  case '!': // Send flight software version
    SERIAL_PRINTLN(VERSION, 2);
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
#elif defined(AeroQuadMega_CHR6DM)
    PrintValueComma(5);
#elif defined(APM_OP_CHR6DM)
    PrintValueComma(6);
#elif defined(AeroQuad_Mini)
    PrintValueComma(2);
#endif    
    // Determine which motor flight configuration for Configurator GUI
#if defined(quadPlusConfig)
    SERIAL_PRINT('0');
#elif defined(quadXConfig)
    SERIAL_PRINT('1');
#elif defined(hexPlusConfig)
    SERIAL_PRINT('2');
#elif defined(hexXConfig)
    SERIAL_PRINT('3');
#endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;  
  case 'e': // Send AREF value
    SERIAL_PRINTLN(aref);
    queryType = 'X';
    break;
  case 'g': // Send magnetometer cal values
#ifdef HeadingMagHold
    SERIAL_PRINT(compass->getMagMax(XAXIS), 2);
    comma();
    SERIAL_PRINT(compass->getMagMin(XAXIS), 2);
    comma();
    SERIAL_PRINT(compass->getMagMax(YAXIS), 2);
    comma();
    SERIAL_PRINT(compass->getMagMin(YAXIS), 2);
    comma();
    SERIAL_PRINT(compass->getMagMax(ZAXIS), 2);
    comma();
    SERIAL_PRINTLN(compass->getMagMin(ZAXIS), 2);
#endif
    queryType = 'X';
    break;
  case '`': // Send Camera values 
#ifdef Camera
    PrintValueComma(camera.getMode());
    PrintValueComma(camera.getCenterPitch());
    PrintValueComma(camera.getCenterRoll());
    PrintValueComma(camera.getCenterYaw());

    SERIAL_PRINT(camera.getmCameraPitch() , 2);
    comma();
    SERIAL_PRINT(camera.getmCameraRoll() , 2);
    comma();
    SERIAL_PRINT(camera.getmCameraYaw() , 2);
    comma();
    PrintValueComma(camera.getServoMinPitch());
    PrintValueComma(camera.getServoMinRoll());
    PrintValueComma(camera.getServoMinYaw());
    PrintValueComma(camera.getServoMaxPitch());
    PrintValueComma(camera.getServoMaxRoll());
    SERIAL_PRINTLN(camera.getServoMaxYaw());
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
    if (SERIAL_AVAILABLE() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = SERIAL_READ();
      timeout = 0;
      index++;
    }
  }  
  while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';
  return atof(data);
}

void comma() {
  SERIAL_PRINT(',');
}

void printInt(int data) {
  byte msb, lsb;

  msb = data >> 8;
  lsb = data & 0xff;

  binaryPort->print(msb, BYTE);
  binaryPort->print(lsb, BYTE);
}

void sendBinaryFloat(float data) {
  union binaryFloatType {
    byte floatByte[4];
    float floatVal;
  } binaryFloat;
  
  binaryFloat.floatVal = data;
  binaryPort->print(binaryFloat.floatByte[3], BYTE);
  binaryPort->print(binaryFloat.floatByte[2], BYTE);
  binaryPort->print(binaryFloat.floatByte[1], BYTE);
  binaryPort->print(binaryFloat.floatByte[0], BYTE);
}

void sendBinaryuslong(unsigned long data) {
  union binaryuslongType {
    byte uslongByte[4];
    unsigned long uslongVal;
  } binaryuslong;
  
  binaryuslong.uslongVal = data;
  binaryPort->print(binaryuslong.uslongByte[3], BYTE);
  binaryPort->print(binaryuslong.uslongByte[2], BYTE);
  binaryPort->print(binaryuslong.uslongByte[1], BYTE);
  binaryPort->print(binaryuslong.uslongByte[0], BYTE);
}

#ifdef BinaryWrite
void fastTelemetry(void)
{
  // **************************************************************
  // ***************** Fast Transfer Of Sensor Data ***************
  // **************************************************************
  // AeroQuad.h defines the output rate to be 10ms
  // Since writing to UART is done by hardware, unable to measure data rate directly
  // Through analysis:  115200 baud = 115200 bits/second = 14400 bytes/second
  // If float = 4 bytes, then 3600 floats/second
  // If 10 ms output rate, then 36 floats/10ms
  // Number of floats written using sendBinaryFloat is 15

  if (armed == ON) {
    #ifdef OpenlogBinaryWrite
       printInt(21845); // Start word of 0x5555
       sendBinaryuslong(currentTime);
//        printInt((int)flightMode);
       for (byte axis = ROLL; axis < LASTAXIS; axis++) sendBinaryFloat(gyro->getRadPerSec(axis));
       for (byte axis = XAXIS; axis < LASTAXIS; axis++) sendBinaryFloat(accel->getMeterPerSec(axis));
//        sendBinaryFloat(accel->accelOneG);
       #ifdef HeadingMagHold
//          sendBinaryFloat(compass->hdgX);
//          sendBinaryFloat(compass->hdgY);
           sendBinaryFloat(compass->getRawData(XAXIS));
           sendBinaryFloat(compass->getRawData(YAXIS));
           sendBinaryFloat(compass->getRawData(ZAXIS));
       #else
         sendBinaryFloat(0.0);
         sendBinaryFloat(0.0);
//          sendBinaryFloat(0.0);
       #endif
//        for (byte axis = ROLL; axis < ZAXIS; axis++) sendBinaryFloat(flightAngle->getData(axis));
       printInt(32767); // Stop word of 0x7FFF
    #else
       printInt(21845); // Start word of 0x5555
       for (byte axis = ROLL; axis < LASTAXIS; axis++) sendBinaryFloat(gyro->getRadPerSec(axis));
       for (byte axis = XAXIS; axis < LASTAXIS; axis++) sendBinaryFloat(accel->getMeterPerSec(axis));
       for (byte axis = ROLL; axis < LASTAXIS; axis++)
       #ifdef HeadingMagHold
         sendBinaryFloat(compass->getRawData(axis));
       #else
         sendBinaryFloat(0);
       #endif
       for (byte axis = ROLL; axis < LASTAXIS; axis++) sendBinaryFloat(flightAngle->getGyroUnbias(axis));
       for (byte axis = ROLL; axis < LASTAXIS; axis++) sendBinaryFloat(flightAngle->getData(axis));
       printInt(32767); // Stop word of 0x7FFF
    #endif
  }
}
#endif    

