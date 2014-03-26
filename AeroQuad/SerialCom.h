/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_SERIAL_COMM_
#define _AQ_SERIAL_COMM_

char queryType = 'X';

void initCommunication() {
  // do nothing here for now
}

//***************************************************************************************************
//********************************** Serial Commands ************************************************
//***************************************************************************************************
bool validateCalibrateCommand(byte command)
{
  if (readFloatSerial() == 123.45) {// use a specific float value to validate full throttle call is being sent
    motorArmed = OFF;
    calibrateESC = command;
    return true;
  }
  else {
    calibrateESC = 0;
    testCommand = 1000;
    return false;
  }
}

void readSerialPID(unsigned char PIDid) {
  struct PIDdata* pid = &PID[PIDid];
  pid->P = readFloatSerial();
  pid->I = readFloatSerial();
  pid->D = readFloatSerial();
  pid->lastError = 0;
  pid->integratedError = 0;
}

void skipSerialValues(byte number) {
  for(byte i=0; i<number; i++) {
    readFloatSerial();
  }
}

void readSerialCommand() {
  // Check for serial message
  if (SERIAL_AVAILABLE()) {
    queryType = SERIAL_READ();
    switch (queryType) {
    case 'A': // Receive roll and pitch rate mode PID
      readSerialPID(RATE_XAXIS_PID_IDX);
      readSerialPID(RATE_YAXIS_PID_IDX);
      rotationSpeedFactor = readFloatSerial();
      writeEEPROM();
      break;

    case 'B': // Receive roll/pitch attitude mode PID
      readSerialPID(ATTITUDE_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_YAXIS_PID_IDX);
      readSerialPID(ATTITUDE_GYRO_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_GYRO_YAXIS_PID_IDX);
      writeEEPROM();
      break;

    case 'C': // Receive yaw PID
      readSerialPID(ZAXIS_PID_IDX);
      readSerialPID(HEADING_HOLD_PID_IDX);
      writeEEPROM();
      break;

    case 'D': // Altitude hold PID
      #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
        readSerialPID(BARO_ALTITUDE_HOLD_PID_IDX);
        readFloatSerial();
        altitudeHoldBump = readFloatSerial();
        altitudeHoldPanicStickMovement = readFloatSerial();
        minThrottleAdjust = readFloatSerial();
        maxThrottleAdjust = readFloatSerial();
        #if defined AltitudeHoldBaro
          baroSmoothFactor = readFloatSerial();
        #else
          readFloatSerial();
        #endif
        readSerialPID(ZDAMPENING_PID_IDX);
        writeEEPROM();
      #endif
      break;

    case 'E': // Receive sensor filtering values
      readFloatSerial();
      minArmedThrottle = readFloatSerial();
      break;

    case 'F': // Receive transmitter smoothing values
      receiverXmitFactor = readFloatSerial();
      for(byte channel = XAXIS; channel<LASTCHANNEL; channel++) {
        receiverSmoothFactor[channel] = readFloatSerial();
      }
      break;

    case 'G': // Receive transmitter calibration values
      for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
        receiverMinValue[axis] = readIntegerSerial();
      }
      break;

    case 'H': // Receive transmitter calibration values
      for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
        receiverMaxValue[axis] = readIntegerSerial();
      }
      writeEEPROM();
      break;

    case 'I': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      writeEEPROM();
      storeSensorsZeroToEEPROM();
      calibrateGyro();
      zeroIntegralError();
      #ifdef HeadingMagHold
        initializeMagnetometer();
      #endif
      #ifdef AltitudeHoldBaro
        initializeBaro();
      #endif
      break;

    case 'J': // calibrate gyros
      calibrateGyro();
      break;

    case 'K': // Write accel calibration values
      accelScaleFactor[XAXIS] = readFloatSerial();
      accelScaleFactor[YAXIS] = readFloatSerial();
      accelScaleFactor[ZAXIS] = readFloatSerial();
      computeAccelBias();    
      storeSensorsZeroToEEPROM();
      break;

    case 'L': // generate accel bias
      computeAccelBias();
      #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
        calibrateKinematics();
        accelOneG = meterPerSecSec[ZAXIS];
      #endif
      storeSensorsZeroToEEPROM();
      break;

    case 'M': // calibrate magnetometer
      #ifdef HeadingMagHold
        magBias[XAXIS] = readFloatSerial();
        magBias[YAXIS] = readFloatSerial();
        magBias[ZAXIS] = readFloatSerial();
        writeEEPROM();
      #else
        skipSerialValues(3);
      #endif
      break;

    case 'N': // battery monitor
      #ifdef BattMonitor
        batteryMonitorAlarmVoltage = readFloatSerial();
        batteryMonitorThrottleTarget = readFloatSerial();
        batteryMonitorGoingDownTime = readFloatSerial();
        setBatteryCellVoltageThreshold(batteryMonitorAlarmVoltage);
        writeEEPROM();
      #else
        skipSerialValues(3);
      #endif
      break;

    case 'O': // define waypoints
      #ifdef UseGPSNavigator
        missionNbPoint = readIntegerSerial();
        waypoint[missionNbPoint].latitude = readIntegerSerial();
        waypoint[missionNbPoint].longitude = readIntegerSerial();
        waypoint[missionNbPoint].altitude = readIntegerSerial();
        writeEEPROM();
      #else
        for(byte i = 0; i < 4; i++) {
          readFloatSerial();
        }
      #endif
      break;
    case 'P': //  read Camera values
      #ifdef CameraControl
        cameraMode = readFloatSerial();
        servoCenterPitch = readFloatSerial();
        servoCenterRoll = readFloatSerial();
        servoCenterYaw = readFloatSerial();
        mCameraPitch = readFloatSerial();
        mCameraRoll = readFloatSerial();
        mCameraYaw = readFloatSerial();
        servoMinPitch = readFloatSerial();
        servoMinRoll = readFloatSerial();
        servoMinYaw = readFloatSerial();
        servoMaxPitch = readFloatSerial();
        servoMaxRoll = readFloatSerial();
        servoMaxYaw = readFloatSerial();
        writeEEPROM();
        #ifdef CameraTXControl
          servoTXChannels = readFloatSerial();
        #endif
      #else
        #ifdef CameraTXControl
          skipSerialValues(14)
        #else
          skipSerialValues(13);
        #endif
      #endif
      break;

    case 'U': // Range Finder
      #if defined (AltitudeHoldRangeFinder)
        maxRangeFinderRange = readFloatSerial();
        minRangeFinderRange = readFloatSerial();
        writeEEPROM();
      #else
        skipSerialValues(2);
      #endif
      break;

    case 'V': // GPS
      #if defined (UseGPSNavigator)
        readSerialPID(GPSROLL_PID_IDX);
        readSerialPID(GPSPITCH_PID_IDX);
        readSerialPID(GPSYAW_PID_IDX);
        writeEEPROM();
      #else
        skipSerialValues(9);
      #endif
      break;

    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;

    case 'X': // Stop sending messages
      break;

    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      validateCalibrateCommand(1);
      break;

    case '2': // Calibrate ESC's by setting Throttle low on all channels
      validateCalibrateCommand(2);
      break;

    case '3': // Test ESC calibration
      if (validateCalibrateCommand(3)) {
        testCommand = readFloatSerial();
      }
      break;

    case '4': // Turn off ESC calibration
      if (validateCalibrateCommand(4)) {
        calibrateESC = 0;
        testCommand = 1000;
      }
      break;

    case '5': // Send individual motor commands (motor, command)
      if (validateCalibrateCommand(5)) {
        for (byte motor = 0; motor < LASTMOTOR; motor++) {
          motorConfiguratorCommand[motor] = (int)readFloatSerial();
        }
      }
      break;

    case 'Z': // fast telemetry transfer <--- get rid if this?
      if (readFloatSerial() == 1.0)
        fastTransfer = ON;
      else
        fastTransfer = OFF;
      break;
    }
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

void PrintValueComma(byte val)
{
  SERIAL_PRINT(val);
  comma();
}

void PrintValueComma(long int val)
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

void PrintDummyValues(byte number) {
  for(byte i=0; i<number; i++) {
    PrintValueComma(0);
  }
}


float getHeading()
{
  #if defined(HeadingMagHold) 
    return trueNorthHeading;
  #else
    return(gyroHeading);
  #endif
}

void sendSerialTelemetry() {
//  Serial.println(degrees(trueNorthHeading));
  
  switch (queryType) {
  case '=': // Reserved debug command to view any variable from Serial Monitor
    break;

  case 'a': // Send roll and pitch rate mode PID values
    PrintPID(RATE_XAXIS_PID_IDX);
    PrintPID(RATE_YAXIS_PID_IDX);
    PrintValueComma(rotationSpeedFactor);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'b': // Send roll and pitch attitude mode PID values
    PrintPID(ATTITUDE_XAXIS_PID_IDX);
    PrintPID(ATTITUDE_YAXIS_PID_IDX);
    PrintPID(ATTITUDE_GYRO_XAXIS_PID_IDX);
    PrintPID(ATTITUDE_GYRO_YAXIS_PID_IDX);
    SERIAL_PRINTLN(0);
    queryType = 'X';
    break;

  case 'c': // Send yaw PID values
    PrintPID(ZAXIS_PID_IDX);
    PrintPID(HEADING_HOLD_PID_IDX);
    SERIAL_PRINTLN(0);
    queryType = 'X';
    break;

  case 'd': // Altitude Hold
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      PrintPID(BARO_ALTITUDE_HOLD_PID_IDX);
      PrintValueComma(0);
      PrintValueComma(altitudeHoldBump);
      PrintValueComma(altitudeHoldPanicStickMovement);
      PrintValueComma(minThrottleAdjust);
      PrintValueComma(maxThrottleAdjust);
      #if defined AltitudeHoldBaro
        PrintValueComma(baroSmoothFactor);
      #else
        PrintValueComma(0);
      #endif
      PrintPID(ZDAMPENING_PID_IDX);
    #else
      PrintDummyValues(10);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'e': // miscellaneous config values
    PrintValueComma(0);
    SERIAL_PRINTLN(minArmedThrottle);
    queryType = 'X';
    break;

  case 'f': // Send transmitter smoothing values
    PrintValueComma(receiverXmitFactor);
    for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
      PrintValueComma(receiverSmoothFactor[axis]);
    }
    PrintDummyValues(10 - LASTCHANNEL);
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'g': // Send transmitter calibration data
    for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
      Serial.print(receiverMinValue[axis]);
      Serial.print(',');
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'h': // Send transmitter calibration data
    for (byte axis = XAXIS; axis < LASTCHANNEL; axis++) {
      Serial.print(receiverMaxValue[axis]);
      Serial.print(',');
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'i': // Send sensor data
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      PrintValueComma(gyroRate[axis]);
    }
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      PrintValueComma(filteredAccel[axis]);
    }
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      #if defined(HeadingMagHold)
        PrintValueComma(getMagnetometerData(axis));
      #else
        PrintValueComma(0);
      #endif
    }
    #if defined(AltitudeHoldBaro)
      PrintValueComma(getBaroAltitude());
      PrintValueComma(zVelocity/100.0F);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
    #endif
    SERIAL_PRINTLN();
    break;

  case 'j': // Send raw mag values
    #ifdef HeadingMagHold
      PrintValueComma(rawMag[XAXIS]);
      PrintValueComma(rawMag[YAXIS]);
      SERIAL_PRINTLN(rawMag[ZAXIS]);
    #endif
    break;

  case 'k': // Send accelerometer cal values
    SERIAL_PRINT(accelScaleFactor[XAXIS], 6);
    comma();
    SERIAL_PRINT(accelScaleFactor[YAXIS], 6);
    comma();
    SERIAL_PRINTLN(accelScaleFactor[ZAXIS], 6);
    queryType = 'X';
    break;

  case 'l': // Send raw accel values
    evaluateMetersPerSec();    // reset sample data
    delay(2);
    measureCriticalSensors();  // call mesureAccelSum that give one raw sample in accelSample
    PrintValueComma((int)(accelSample[XAXIS]));
    PrintValueComma((int)(accelSample[YAXIS]));
    SERIAL_PRINTLN ((int)(accelSample[ZAXIS]));
    break;

  case 'm': // Send magnetometer cal values
    #ifdef HeadingMagHold
      SERIAL_PRINT(magBias[XAXIS], 6);
      comma();
      SERIAL_PRINT(magBias[YAXIS], 6);
      comma();
      SERIAL_PRINTLN(magBias[ZAXIS], 6);
    #endif
    queryType = 'X';
    break;

  case 'n': // battery monitor
    #ifdef BattMonitor
      PrintValueComma(batteryMonitorAlarmVoltage);
      PrintValueComma(batteryMonitorThrottleTarget);
      PrintValueComma(batteryMonitorGoingDownTime);
    #else
      PrintDummyValues(3);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'o': // send waypoints
    #ifdef UseGPSNavigator
      for (byte index = 0; index < MAX_WAYPOINTS; index++) {
        PrintValueComma(index);
        PrintValueComma(waypoint[index].latitude);
        PrintValueComma(waypoint[index].longitude);
        PrintValueComma(waypoint[index].altitude);
      }
    #else
      PrintDummyValues(4);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'p': // Send Camera values
    #ifdef CameraControl
      PrintValueComma(cameraMode);
      PrintValueComma(servoCenterPitch);
      PrintValueComma(servoCenterRoll);
      PrintValueComma(servoCenterYaw);
      PrintValueComma(mCameraPitch);
      PrintValueComma(mCameraRoll);
      PrintValueComma(mCameraYaw);
      PrintValueComma(servoMinPitch);
      PrintValueComma(servoMinRoll);
      PrintValueComma(servoMinYaw);
      PrintValueComma(servoMaxPitch);
      PrintValueComma(servoMaxRoll);
      PrintValueComma(servoMaxYaw);
      #ifdef CameraTXControl
        PrintValueComma(servoTXChannels);
      #endif
    #else
      #ifdef CameraTXControl
        PrintDummyValues(14);
      #else
        PrintDummyValues(13);
      #endif
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

  case 'q': // Send Vehicle State Value
    SERIAL_PRINTLN(vehicleState);
    queryType = 'X';
    break;

  case 'r': // Vehicle attitude
    PrintValueComma(kinematicsAngle[XAXIS]);
    PrintValueComma(kinematicsAngle[YAXIS]);
    SERIAL_PRINTLN(getHeading());
    break;

  case 's': // Send all flight data
    PrintValueComma(motorArmed);
    PrintValueComma(kinematicsAngle[XAXIS]);
    PrintValueComma(kinematicsAngle[YAXIS]);
    PrintValueComma(getHeading());
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      #if defined AltitudeHoldBaro
        PrintValueComma(estimatedAltitude);
        PrintValueComma(zVelocity/100.0F);
      #elif defined AltitudeHoldRangeFinder
        PrintValueComma(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_RANGE ? rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] : 0.0);
        PrintValueComma(0);
      #endif
      PrintValueComma((int)altitudeHoldState);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
      PrintValueComma(0);
    #endif

    for (byte channel = 0; channel < 8; channel++) { // Configurator expects 8 values
      PrintValueComma((channel < LASTCHANNEL) ? receiverCommand[channel] : 0);
    }

    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    PrintDummyValues(8 - LASTMOTOR); // max of 8 motor outputs supported

    #if defined(BattMonitor)
      PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
    #else
      PrintValueComma(0);
    #endif
    
    PrintValueComma(flightMode);
    
    #if defined(UseGPS)
      PrintValueComma(gpsData.state);
      PrintValueComma(gpsData.sats);
      PrintValueComma(gpsData.speed);
      PrintValueComma(gpsData.height);
      PrintValueComma(gpsData.course);
      PrintValueComma(gpsData.lat);
      PrintValueComma(gpsData.lon);
    #else
      PrintValueComma(0);
      PrintValueComma(0);
      PrintValueComma(0);
      PrintValueComma(0);
      PrintValueComma(0);
      PrintValueComma(0);
      PrintValueComma(0);
    #endif
    
    SERIAL_PRINTLN();
    break;

  case 't': // Send processed transmitter values
    for (byte axis = 0; axis < LASTCHANNEL; axis++) {
      PrintValueComma(getRawChannelValue(axis));
    }
    SERIAL_PRINTLN();
    break;

  case 'u': // Send range finder values
    #if defined (AltitudeHoldRangeFinder)
      PrintValueComma(maxRangeFinderRange);
      SERIAL_PRINTLN(minRangeFinderRange);
    #else
      PrintValueComma(0);
      SERIAL_PRINTLN(0);
    #endif
    queryType = 'X';
    break;

  case 'v': // Send GPS PIDs
    #if defined (UseGPSNavigator)
      PrintPID(GPSROLL_PID_IDX);
      PrintPID(GPSPITCH_PID_IDX);
      PrintPID(GPSYAW_PID_IDX);
      queryType = 'X';
    #else
      PrintDummyValues(9);
    #endif
    SERIAL_PRINTLN();
    queryType = 'X';
    break;
  case 'y': // send GPS info
    #if defined (UseGPS)
      PrintValueComma(gpsData.state);
      PrintValueComma(gpsData.lat);
      PrintValueComma(gpsData.lon);
      PrintValueComma(gpsData.height);
      PrintValueComma(gpsData.course);
      PrintValueComma(gpsData.speed);
      PrintValueComma(gpsData.accuracy);
      PrintValueComma(gpsData.sats);
      PrintValueComma(gpsData.fixtime);
      PrintValueComma(gpsData.sentences);
      PrintValueComma(gpsData.idlecount);
    #else
      PrintDummyValues(11);
    #endif    
    SERIAL_PRINTLN();
    break;
 
  case 'z': // Send all Altitude data 
    #if defined (AltitudeHoldBaro) 
      PrintValueComma(getBaroAltitude()); 
    #else
      PrintValueComma(0);
    #endif 
    #if defined (AltitudeHoldRangeFinder) 
      SERIAL_PRINTLN(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
    #else
      SERIAL_PRINTLN(0); 
    #endif 
    break;
    
  case '$': // send BatteryMonitor voltage/current readings
    #if defined (BattMonitor)
      PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
      #if defined (BM_EXTENDED)
        PrintValueComma((float)batteryData[0].current/100.0);
        PrintValueComma((float)batteryData[0].usedCapacity/1000.0);
      #else
        PrintDummyValues(2);
      #endif
    #else
      PrintDummyValues(3);
    #endif
    SERIAL_PRINTLN();
    break;
    
  case '%': // send RSSI
    #if defined (UseAnalogRSSIReader) || defined (UseEzUHFRSSIReader) || defined (UseSBUSRSSIReader)
      SERIAL_PRINTLN(rssiRawValue);
    #else
      SERIAL_PRINTLN(0);
    #endif
    break;

  case 'x': // Stop sending messages
    break;

  case '!': // Send flight software version
    SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
    queryType = 'X';
    break;

  case '#': // Send configuration
    reportVehicleState();
    queryType = 'X';
    break;

  case '6': // Report remote commands
    for (byte motor = 0; motor < LASTMOTOR; motor++) {
      PrintValueComma(motorCommand[motor]);
    }
    SERIAL_PRINTLN();
    queryType = 'X';
    break;

#if defined(OSD) && defined(OSD_LOADFONT)
  case '&': // fontload
    if (OFF == motorArmed) {
      max7456LoadFont();
    }
    queryType = 'X';
    break;
#endif

  }
}

void readValueSerial(char *data, byte size) {
  byte index = 0;
  byte timeout = 0;
  data[0] = '\0';

  do {
    if (SERIAL_AVAILABLE() == 0) {
      delay(1);
      timeout++;
    } else {
      data[index] = SERIAL_READ();
      timeout = 0;
      index++;
    }
  } while ((index == 0 || data[index-1] != ';') && (timeout < 10) && (index < size-1));

  data[index] = '\0';
}


// Used to read floating point values from the serial port
float readFloatSerial() {
  char data[15] = "";

  readValueSerial(data, sizeof(data));
  return atof(data);
}

// Used to read integer values from the serial port
long readIntegerSerial() {
  char data[16] = "";

  readValueSerial(data, sizeof(data));
  return atol(data);
}

void comma() {
  SERIAL_PRINT(',');
}

void printVehicleState(const char *sensorName, unsigned long state, const char *message) {
  
  SERIAL_PRINT(sensorName);
  SERIAL_PRINT(": ");
  if (!(vehicleState & state)) {
    SERIAL_PRINT("Not ");
  }
  SERIAL_PRINTLN(message);
}

void reportVehicleState() {
  // Tell Configurator how many vehicle state values to expect
  SERIAL_PRINTLN(15);
  SERIAL_PRINT("Software Version: ");
  SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
  SERIAL_PRINT("Board Type: ");
  #if defined(AeroQuad_v18)
    SERIAL_PRINTLN("v1.8 and greater");
  #elif defined(AeroQuad_Mini)
    SERIAL_PRINTLN("Mini");
  #elif defined(MWCFlip15)
    SERIAL_PRINTLN("MWCFlip15");
  #elif defined(AeroQuadMega_v2)
    SERIAL_PRINTLN("Mega v2");
  #elif defined(AeroQuadMega_v21)
    SERIAL_PRINTLN("Mega v21");
  #elif defined(MWCProEz30)
    SERIAL_PRINTLN("MWCProEz30");    
  #elif defined(AeroQuadSTM32)
    SERIAL_PRINTLN(STM32_BOARD_TYPE);
  #endif

  SERIAL_PRINT("Flight Config: ");
  #if defined(quadPlusConfig)
    SERIAL_PRINTLN("Quad +");
  #elif defined(quadXConfig)
    SERIAL_PRINTLN("Quad X");
  #elif defined (quadY4Config)
    SERIAL_PRINTLN("Quad Y4");
  #elif defined (triConfig)
    SERIAL_PRINTLN("Tri");
  #elif defined(hexPlusConfig)
    SERIAL_PRINTLN("Hex +");
  #elif defined(hexXConfig)
    SERIAL_PRINTLN("Hex X");
  #elif defined(hexY6Config)
    SERIAL_PRINTLN("Hex Y6");
  #elif defined(octoX8Config)
    SERIAL_PRINTLN("Octo X8");
  #elif defined(octoXConfig)
    SERIAL_PRINTLN("Octo X");
  #elif defined(octoPlusConfig)
    SERIAL_PRINTLN("Octo +");
  #endif

  SERIAL_PRINT("Receiver Channels: ");
  SERIAL_PRINTLN(LASTCHANNEL);

  SERIAL_PRINT("Motors: ");
  SERIAL_PRINTLN(LASTMOTOR);

  printVehicleState("Gyroscope", GYRO_DETECTED, "Detected");
  printVehicleState("Accelerometer", ACCEL_DETECTED, "Detected");
  printVehicleState("Barometer", BARO_DETECTED, "Detected");
  printVehicleState("Magnetometer", MAG_DETECTED, "Detected");
  printVehicleState("Heading Hold", HEADINGHOLD_ENABLED, "Enabled");
  printVehicleState("Altitude Hold", ALTITUDEHOLD_ENABLED, "Enabled");
  printVehicleState("Battery Monitor", BATTMONITOR_ENABLED, "Enabled");
  printVehicleState("Camera Stability", CAMERASTABLE_ENABLED, "Enabled");
  printVehicleState("Range Detection", RANGE_ENABLED, "Enabled");
#ifdef UseGPS
  SERIAL_PRINTLN("GPS: Enabled");
#else
  SERIAL_PRINTLN("GPS: Not Enabled");
#endif
}

#endif // _AQ_SERIAL_COMM_
