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

// SerialCom.pde is responsible for the serial communication for commands and telemetry from the AeroQuad
// This comtains readSerialCommand() which listens for a serial command and it's arguments
// This also contains readSerialTelemetry() which listens for a telemetry request and responds with the requested data
// For more information on each command/telemetry look at: http://aeroquad.com/content.php?117

// Includes re-write / fixes from Aadamson and ala42, special thanks to those guys!
// http://aeroquad.com/showthread.php?1461-We-have-some-hidden-warnings&p=14618&viewfull=1#post14618

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
void readSerialPID(unsigned char PIDid) 
{
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastPosition = 0;
  pid->integratedError = 0;
}

void readSerialCommand() 
{
  // Check for serial message
  if (Serial.available()) 
  {
    digitalWrite(LEDPIN, LOW);
    _queryType = Serial.read();
    switch (_queryType) 
    {
    case 'A': // Receive roll and pitch gyro PID
      readSerialPID(ROLL);
      readSerialPID(PITCH);
      _minAcro = readFloatSerial();
      break;
    case 'C': // Receive yaw PID
      readSerialPID(YAW);
      readSerialPID(HEADING);
      _headingHoldConfig = readFloatSerial();
      _heading = 0;
      _relativeHeading = 0;
      _headingHold = 0;
      break;
    case 'E': // Receive roll and pitch auto level PID
      readSerialPID(LEVELROLL);
      readSerialPID(LEVELPITCH);
      readSerialPID(LEVELGYROROLL);
      readSerialPID(LEVELGYROPITCH);
      windupGuard = readFloatSerial();
      break;
    case 'G': // Receive auto level configuration
      _levelLimit = readFloatSerial();
      _levelOff = readFloatSerial();
      break;
    case 'I': // Receiver altitude hold PID
#ifdef AltitudeHold
      readSerialPID(ALTITUDE);
      PID[ALTITUDE].windupGuard = readFloatSerial();
      _minThrottleAdjust = readFloatSerial();
      _maxThrottleAdjust = readFloatSerial();
      _altitude->setSmoothFactor(readFloatSerial());
      readSerialPID(ZDAMPENING);
#endif
      break;
    case 'K': // Receive data filtering values
      _gyro->setSmoothFactor(readFloatSerial());
      _accel->setSmoothFactor(readFloatSerial());
      _timeConstant = readFloatSerial();
#if defined(AeroQuad_v1) || defined(AeroQuad_v18)
      _flightAngle->initialize();
#endif
      break;
    case 'M': // Receive transmitter smoothing values
      _receiver->setXmitFactor(readFloatSerial());
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) 
      {
        _receiver->setSmoothFactor(channel, readFloatSerial());
      }
      break;
    case 'O': // Receive transmitter calibration values
      for(byte channel = ROLL; channel<LASTCHANNEL; channel++) 
      {
        _receiver->setTransmitterSlope(channel, readFloatSerial());
        _receiver->setTransmitterOffset(channel, readFloatSerial());
      }
      break;
    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;
    case 'Y': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      _gyro->calibrate();
      _accel->calibrate();
      storeSensorsToEEPROM();
      zeroIntegralError();
#ifdef HeadingMagHold
      _compass->initialize();
#endif
#ifdef AltitudeHold
      _altitude->initialize();
#endif
      break;
    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      _armed = 0;
      _calibrateESC = 1;
      break;
    case '2': // Calibrate ESC's by setting Throttle low on all channels
      _armed = 0;
      _calibrateESC = 2;
      break;
    case '3': // Test ESC calibration
      _armed = 0;
      _testCommand = readFloatSerial();
      _calibrateESC = 3;
      break;
    case '4': // Turn off ESC calibration
      _armed = 0;
      _calibrateESC = 0;
      _testCommand = 1000;
      break;
    case '5': // Send individual motor commands (motor, command)
      _armed = 0;
      _calibrateESC = 5;
      for (byte motor = FRONT; motor < LASTMOTOR; motor++)
      {
        _motors->setRemoteCommand(motor, readFloatSerial());
      }
      break;
    case 'a':
      // spare
      break;
    case 'b': // calibrate gyros
      _gyro->calibrate();
      storeSensorsToEEPROM();
      break;
    case 'c': // calibrate accels
      _accel->calibrate();
      storeSensorsToEEPROM();
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      _flightAngle->calibrate();
      _accel->setOneG(_accel->getFlightData(ZAXIS));
#endif
      break;
    case 'd': // send aref
      aref = readFloatSerial();
      break;
    case 'f': // calibrate magnetometer
#ifdef HeadingMagHold
      _compass->setMagCal(XAXIS, readFloatSerial(), readFloatSerial());
      _compass->setMagCal(YAXIS, readFloatSerial(), readFloatSerial());
      _compass->setMagCal(ZAXIS, readFloatSerial(), readFloatSerial());
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
  Serial.print(val);
  comma();
}

void PrintValueComma(char val)
{
  Serial.print(val);
  comma();
}

void PrintValueComma(int val)
{
  Serial.print(val);
  comma();
}

void PrintValueComma(unsigned long val)
{
  Serial.print(val);
  comma();
}

void PrintPID(unsigned char IDPid)
{
  PrintValueComma(PID[IDPid].P);
  PrintValueComma(PID[IDPid].I);
  PrintValueComma(PID[IDPid].D);
}

void sendSerialTelemetry() 
{
  _update = 0;
  switch (_queryType) 
  {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    //printFreeMemory();
    Serial.print(_receiver->getAngle(ROLL));
    comma();
    Serial.print(_receiver->getAngle(PITCH));
    Serial.println();
    //queryType = 'X';
    break;
  case 'B': // Send roll and pitch gyro PID values
    PrintPID(ROLL);
    PrintPID(PITCH);
    Serial.println(_minAcro);
    _queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    PrintPID(YAW);
    PrintPID(HEADING);
    Serial.println(_headingHoldConfig, BIN);
    _queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    PrintPID(LEVELROLL);
    PrintPID(LEVELPITCH);
    PrintPID(LEVELGYROROLL);
    PrintPID(LEVELGYROPITCH);
    Serial.println(windupGuard);
    _queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    //Serial.print(levelLimit);
    //comma();
    PrintValueComma(_levelLimit);
    Serial.println(_levelOff);
    _queryType = 'X';
    break;
  case 'J': // Altitude Hold
#ifdef AltitudeHold
    PrintPID(ALTITUDE);
    PrintValueComma(PID[ALTITUDE].windupGuard);
    PrintValueComma(_minThrottleAdjust);
    PrintValueComma(_maxThrottleAdjust);
    PrintValueComma(_altitude->getSmoothFactor());
    PrintValueComma(PID[ZDAMPENING].P);
    PrintValueComma(PID[ZDAMPENING].I);
    Serial.println(PID[ZDAMPENING].D);
#else
    for(byte i=0; i<9; i++) 
    {
     PrintValueComma(0);
    }
    Serial.println('0');
#endif
    _queryType = 'X';
    break;
  case 'L': // Send data filtering values
    PrintValueComma(_gyro->getSmoothFactor());
    PrintValueComma(_accel->getSmoothFactor());
    Serial.println(_timeConstant);
    // comma();
    // Serial.println(flightMode, DEC);
    _queryType = 'X';
    break;
  case 'N': // Send transmitter smoothing values
    PrintValueComma(_receiver->getXmitFactor());
    for (byte axis = ROLL; axis < AUX; axis++) 
    {
      PrintValueComma(_receiver->getSmoothFactor(axis));
    }
    Serial.println(_receiver->getSmoothFactor(AUX));
    _queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (byte axis = ROLL; axis < AUX; axis++) 
    {
      PrintValueComma(_receiver->getTransmitterSlope(axis));
      PrintValueComma(_receiver->getTransmitterOffset(axis));
    }
    PrintValueComma(_receiver->getTransmitterSlope(AUX));
    Serial.println(_receiver->getTransmitterOffset(AUX));
    _queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      PrintValueComma(_gyro->getData(axis));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      PrintValueComma(_accel->getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) 
    {
      PrintValueComma(_levelAdjust[axis]);
    }
    PrintValueComma(_flightAngle->getData(ROLL));
    PrintValueComma(_flightAngle->getData(PITCH));
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma(_compass->getAbsoluteHeading());
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(_altitude->getData());
    #else
      PrintValueComma(0);
    #endif
    #ifdef BattMonitor
      Serial.print(_batteryMonitor->getData());
    #else
      Serial.print(0);
    #endif
    Serial.println();
    break;
  case 'R': // Raw magnetometer data
#if defined(HeadingMagHold)
    PrintValueComma(_compass->getRawData(XAXIS));
    PrintValueComma(_compass->getRawData(YAXIS));
    Serial.println(_compass->getRawData(ZAXIS));
#else
    PrintValueComma(0);
    PrintValueComma(0);
    Serial.println('0');
#endif
    break;
  case 'S': // Send all flight data
    PrintValueComma(_deltaTime);
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      PrintValueComma(_gyro->getFlightData(axis));
    }
    #ifdef BattMonitor
      PrintValueComma(_batteryMonitor->getData());
    #else
      PrintValueComma(0);
    #endif
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      PrintValueComma(_motors->getMotorAxisCommand(axis));
    }
    for (byte motor = FRONT; motor < LASTMOTOR; motor++) 
    {
      PrintValueComma(_motors->getMotorCommand(motor));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      PrintValueComma(_accel->getFlightData(axis));
    }
    Serial.print(_armed, BIN);
    comma();
    if (_flightMode == STABLE)
    {
      PrintValueComma(2000);
    }
    if (_flightMode == ACRO)
    {
      PrintValueComma(1000);
    }
    #ifdef HeadingMagHold
      PrintValueComma(_compass->getAbsoluteHeading());
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(_altitude->getData());
      Serial.print(_altitudeHold, DEC);
    #else
      PrintValueComma(0);
      Serial.print('0');
    #endif
    Serial.println();    
    break;
  case 'T': // Send processed transmitter values
    PrintValueComma(_receiver->getXmitFactor());
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      PrintValueComma(_receiver->getData(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) 
    {
      PrintValueComma(_levelAdjust[axis]);
    }
    PrintValueComma(_motors->getMotorAxisCommand(ROLL));
    PrintValueComma(_motors->getMotorAxisCommand(PITCH));
    Serial.println(_motors->getMotorAxisCommand(YAW));
    break;
  case 'U': // Send smoothed receiver with Transmitter Factor applied values
    for (byte channel = ROLL; channel < AUX; channel++) 
    {
      PrintValueComma(_receiver->getData(channel));
    }
    Serial.println(_receiver->getData(AUX));
    break;
  case 'V': // Send receiver status
    for (byte channel = ROLL; channel < AUX; channel++) 
    {
      PrintValueComma(_receiver->getRaw(channel));
    }
    Serial.println(_receiver->getRaw(AUX));
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    PrintValueComma(_receiver->getData(YAW));
    PrintValueComma(_headingHold);
    PrintValueComma(_setHeading);
    Serial.println(_relativeHeading);
    break;
  case '6': // Report remote commands
    for (byte motor = FRONT; motor < LEFT; motor++) 
    {
      PrintValueComma(_motors->getRemoteCommand(motor));
    }
    Serial.println(_motors->getRemoteCommand(LEFT));
    break;
  case '!': // Send flight software version
    Serial.println(VERSION, 1);
    _queryType = 'X';
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
    Serial.print('0');
#elif defined(XConfig)
    Serial.print('1');
#elif defined(HEXACOAXIAL)
    Serial.print('2');
#elif defined(HEXARADIAL)
    Serial.print('3');
#endif
    Serial.println();
    _queryType = 'X';
    break;  
  case 'e': // Send AREF value
    Serial.println(aref);
    _queryType = 'X';
    break;
  case 'g': // Send magnetometer cal values
#ifdef HeadingMagHold
    Serial.print(_compass->getMagMax(XAXIS), 2);
    comma();
    Serial.print(_compass->getMagMin(XAXIS), 2);
    comma();
    Serial.print(_compass->getMagMax(YAXIS), 2);
    comma();
    Serial.print(_compass->getMagMin(YAXIS), 2);
    comma();
    Serial.print(_compass->getMagMax(ZAXIS), 2);
    comma();
    Serial.println(_compass->getMagMin(ZAXIS), 2);
#endif
    _queryType = 'X';
    break;
  case '`': // Send Camera values 
    #ifdef Camera
    //Serial.print(camera.getMode());
    //comma();
    PrintValueComma(camera.getMode());
    //Serial.print(camera.getCenterPitch());
    //comma();
    PrintValueComma(camera.getCenterPitch());
    //Serial.print(camera.getCenterRoll());
    //comma();
    PrintValueComma(camera.getCenterRoll());
    //Serial.print(camera.getCenterYaw());
    //comma();
    PrintValueComma(camera.getCenterYaw());
    Serial.print(camera.getmCameraPitch() , 2);
    comma();
    Serial.print(camera.getmCameraRoll() , 2);
    comma();
    Serial.print(camera.getmCameraYaw() , 2);
    comma();
    //Serial.print(camera.getServoMinPitch());
    //comma();
    PrintValueComma(camera.getServoMinPitch());
    //Serial.print(camera.getServoMinRoll());
    //comma();
    PrintValueComma(camera.getServoMinRoll());
    //Serial.print(camera.getServoMinYaw());
    //comma();
    PrintValueComma(camera.getServoMinYaw());
    //Serial.print(camera.getServoMaxPitch());
    //comma();
    PrintValueComma(camera.getServoMaxPitch());
    //Serial.print(camera.getServoMaxRoll());
    //comma();
    PrintValueComma(camera.getServoMaxRoll());
    Serial.println(camera.getServoMaxYaw());
    #endif
    break;
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() 
{
  #define SERIALFLOATSIZE 10
  byte index = 0;
  byte timeout = 0;
  char data[SERIALFLOATSIZE] = "";

  do 
  {
    if (Serial.available() == 0) 
    {
      delay(10);
      timeout++;
    }
    else 
    {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  
  while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';
  return atof(data);
}

void comma() 
{
  Serial.print(',');
}

void printInt(int data) 
{
  byte msb, lsb;

  msb = data >> 8;
  lsb = data & 0xff;

  Serial.print(msb, BYTE);
  Serial.print(lsb, BYTE);
}

