/*
  AeroQuad v2.3 - February 2011
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
}

void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = Serial.read();
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
      //_flightAngle->initialize();
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
      _flightAngle->calibrate();
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

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    /*
    Serial.print("HoldA: ");
    Serial.print(holdAltitude);
    Serial.print(" Altitude: ");
    Serial.print(altitude.getData());
    Serial.print(" HoldT: ");
    Serial.print(holdThrottle);
    Serial.print(" rxThrottle: ");
    Serial.print(receiver.getData(THROTTLE));
    Serial.print(" rxAUX: ");
    Serial.print(receiver.getRaw(AUX));
    Serial.print(" ThottleA: ");
    Serial.print(throttleAdjust);*/
    PrintValueComma((float)RAD_2_DEG(kinematics.getDriftCorrectedRate(ROLL)));
    Serial.print(gyro.getFlightData(ROLL));
    Serial.println();
    //printFreeMemory();
    //queryType = 'X';
    break;
  case 'B': // Send roll and pitch gyro PID values
    PrintPID(ROLL);
    PrintPID(PITCH);
    Serial.println(minAcro);
    queryType = 'X';
    break;
  case 'D': // Send yaw PID values
    PrintPID(YAW);
    PrintPID(HEADING);
    Serial.println(headingHoldConfig, BIN);
    queryType = 'X';
    break;
  case 'F': // Send roll and pitch auto level PID values
    PrintPID(LEVELROLL);
    PrintPID(LEVELPITCH);
    PrintPID(LEVELGYROROLL);
    PrintPID(LEVELGYROPITCH);
    Serial.println(windupGuard);
    queryType = 'X';
    break;
  case 'H': // Send auto level configuration values
    //Serial.print(levelLimit);
    //comma();
		PrintValueComma(levelLimit);
    Serial.println(levelOff);
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
    Serial.println(PID[ZDAMPENING].D);
#else
    for(byte i=0; i<9; i++) {
     PrintValueComma(0);
    }
    Serial.println('0');
#endif
    queryType = 'X';
    break;
  case 'L': // Send data filtering values
    PrintValueComma(gyro.getSmoothFactor());
    PrintValueComma(accel.getSmoothFactor());
    Serial.println(timeConstant);
    // comma();
    // Serial.println(flightMode, DEC);
    queryType = 'X';
    break;
  case 'N': // Send transmitter smoothing values
    PrintValueComma(receiver.getXmitFactor());
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver.getSmoothFactor(axis));
    }
    Serial.println(receiver.getSmoothFactor(AUX));
    queryType = 'X';
    break;
  case 'P': // Send transmitter calibration data
    for (byte axis = ROLL; axis < AUX; axis++) {
      PrintValueComma(receiver.getTransmitterSlope(axis));
      PrintValueComma(receiver.getTransmitterOffset(axis));
    }
    PrintValueComma(receiver.getTransmitterSlope(AUX));
    Serial.println(receiver.getTransmitterOffset(AUX));
    queryType = 'X';
    break;
  case 'Q': // Send sensor data
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma((float)RAD_2_DEG(kinematics.getDriftCorrectedRate(axis)));
    }
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      PrintValueComma(accel.getRaw(axis));
    }
    for (byte axis = ROLL; axis < YAW; axis++) {
      PrintValueComma(levelAdjust[axis]);
    }
    PrintValueComma((float)RAD_2_DEG(kinematics.getAttitude(ROLL)));
    PrintValueComma((float)RAD_2_DEG(kinematics.getAttitude(PITCH)));
    #if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
      PrintValueComma((float)(RAD_2_DEG(kinematics.getAttitude(YAW))));    // jihlein: remove float(RAD_2_DEG()) when configurator is updated to accept radians as input, displayed as degrees
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(altitude.getData());
    #else
      PrintValueComma(0);
    #endif
    #ifdef BattMonitor
      Serial.print(batteryMonitor.getData());
    #else
      Serial.print(0);
    #endif
    Serial.println();
    break;
  case 'R': // Raw magnetometer data
#if defined(HeadingMagHold)
    PrintValueComma(compass.getRawData(XAXIS));
    PrintValueComma(compass.getRawData(YAXIS));
    Serial.println(compass.getRawData(ZAXIS));
#else
    PrintValueComma(0);
    PrintValueComma(0);
    Serial.println('0');
#endif
    break;
  case 'S': // Send all flight data
    PrintValueComma(deltaTime);
    for (byte axis = ROLL; axis < LASTAXIS; axis++) {
      //PrintValueComma((float)(RAD_2_DEG(kinematics.getDriftCorrectedRate(axis))) * 2);
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
      PrintValueComma(accel.getRaw(axis));
    }
    Serial.print(armed, BIN);
    comma();
    if (flightMode == STABLE)
      PrintValueComma(2000);
    if (flightMode == ACRO)
      PrintValueComma(1000);
    #ifdef HeadingMagHold
      PrintValueComma((float)(RAD_2_DEG(kinematics.getAttitude(YAW))));  // jihlein: remove float(RAD_2_DEG()) when configurator is updated to accept radians as input, displayed as degrees
    #else
      PrintValueComma(0);
    #endif
    #ifdef AltitudeHold
      PrintValueComma(altitude.getData());
      Serial.print(altitudeHold, DEC);
    #else
      PrintValueComma(0);
      Serial.print('0');
    #endif
    Serial.println();    
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
    Serial.println(motors.getMotorAxisCommand(YAW));
    break;
  case 'U': // Send smoothed receiver with Transmitter Factor applied values
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver.getData(channel));
    }
    Serial.println(receiver.getData(AUX));
    break;
  case 'V': // Send receiver status
    for (byte channel = ROLL; channel < AUX; channel++) {
      PrintValueComma(receiver.getRaw(channel));
    }
    Serial.println(receiver.getRaw(AUX));
    break;
  case 'X': // Stop sending messages
    break;
  case 'Z': // Send heading
    PrintValueComma(receiver.getData(YAW));
    PrintValueComma(headingHold);
    PrintValueComma(setHeading);
    Serial.println(relativeHeading);
    break;
  case '6': // Report remote commands
    for (byte motor = FRONT; motor < LEFT; motor++) {
      PrintValueComma(motors.getRemoteCommand(motor));
    }
    Serial.println(motors.getRemoteCommand(LEFT));
    break;
  case '!': // Send flight software version
    Serial.println(VERSION, 1);
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
    Serial.print('0');
#elif defined(XConfig)
    Serial.print('1');
#elif defined(HEXACOAXIAL)
    Serial.print('2');
#elif defined(HEXARADIAL)
    Serial.print('3');
#endif
    Serial.println();
    queryType = 'X';
    break;  
  case 'e': // Send AREF value
    Serial.println(aref);
    queryType = 'X';
    break;
  case 'g': // Send magnetometer cal values
#ifdef HeadingMagHold
    Serial.print(compass.getMagMax(XAXIS), 2);
    comma();
    Serial.print(compass.getMagMin(XAXIS), 2);
    comma();
    Serial.print(compass.getMagMax(YAXIS), 2);
    comma();
    Serial.print(compass.getMagMin(YAXIS), 2);
    comma();
    Serial.print(compass.getMagMax(ZAXIS), 2);
    comma();
    Serial.println(compass.getMagMin(ZAXIS), 2);
#endif
    queryType = 'X';
    break;
  case '`': // Send Camera values 
    #ifdef Camera
    PrintValueComma(camera.getMode());
    PrintValueComma(camera.getCenterPitch());
    PrintValueComma(camera.getCenterRoll());
    PrintValueComma(camera.getCenterYaw());

    Serial.print(camera.getmCameraPitch() , 2);
    comma();
    Serial.print(camera.getmCameraRoll() , 2);
    comma();
    Serial.print(camera.getmCameraYaw() , 2);
    comma();

    PrintValueComma(camera.getServoMinPitch());
    PrintValueComma(camera.getServoMinRoll());
    PrintValueComma(camera.getServoMinYaw());
    PrintValueComma(camera.getServoMaxPitch());
    PrintValueComma(camera.getServoMaxRoll());
    Serial.println(camera.getServoMaxYaw());
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
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  
  while ((index == 0 || data[index-1] != ';') && (timeout < 5) && (index < sizeof(data)-1));
  data[index] = '\0';
  return atof(data);
}

void comma() {
  Serial.print(',');
}

void printInt(int data) {
  byte msb, lsb;

  msb = data >> 8;
  lsb = data & 0xff;

  Serial.print(msb, BYTE);
  Serial.print(lsb, BYTE);
}

