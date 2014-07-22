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
    serialQueryType = SERIAL_READ();
    switch (serialQueryType) {
    case 'A': // Receive roll and pitch rate mode PID
      readSerialPID(RATE_XAXIS_PID_IDX);
      readSerialPID(RATE_YAXIS_PID_IDX);
      userRateRollP = PID[RATE_XAXIS_PID_IDX].P;
      userRateRollD = PID[RATE_XAXIS_PID_IDX].D;
      userRatePitchP = PID[RATE_YAXIS_PID_IDX].P;
      userRatePitchD = PID[RATE_YAXIS_PID_IDX].D;
      rotationSpeedFactor = readFloatSerial();
      throttlePIDAdjustmentFactor = readFloatSerial();
      writeEEPROM();
      break;

    case 'B': // Receive roll/pitch attitude mode PID
      readSerialPID(ATTITUDE_XAXIS_PID_IDX);
      readSerialPID(ATTITUDE_YAXIS_PID_IDX);
      writeEEPROM();
      break;

    case 'C': // Receive yaw PID
      readSerialPID(ZAXIS_PID_IDX);
      yawSpeedFactor = readFloatSerial();
      writeEEPROM();
      break;

    #if defined AltitudeHoldBaro
      case 'D': // Altitude hold PID
          readSerialPID(BARO_ALTITUDE_HOLD_PID_IDX);
          altitudeHoldBump = readFloatSerial();
          altitudeHoldMaxVelocitySpeed = readFloatSerial();
          baroSmoothFactor = readFloatSerial();
          readSerialPID(ZDAMPENING_PID_IDX);
          writeEEPROM();
          break;
    #endif      
    
    #if defined (BattMonitor)
      case 'E':
        isBatteryMonitorEnabled = readFloatSerial();
        writeEEPROM();
        break;
    #endif

    case 'G': // Receive transmitter calibration values
      for (int channel = 0; channel < LAST_CHANNEL; channel++) {
        receiverMinValue[channel] = readFloatSerial();
        receiverMaxValue[channel] = readFloatSerial();
      }
      writeEEPROM();
      break;

    case 'H': // Reset transmitter calibration values
      for (byte channel = XAXIS; channel < LAST_CHANNEL; channel++) {
        receiverMinValue[channel] = 1000;
        receiverMaxValue[channel] = 2000;
      }
      writeEEPROM();
      break;

    case 'I': // Initialize EEPROM with default values
      initializeEEPROM(); // defined in DataStorage.h
      writeEEPROM();
      storeSensorsZeroToEEPROM();
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
      storeSensorsZeroToEEPROM();
      break;

    #ifdef HeadingMagHold
      case 'M': // calibrate magnetometer
          magBias[XAXIS] = readFloatSerial();
          magBias[YAXIS] = readFloatSerial();
          magBias[ZAXIS] = readFloatSerial();
          writeEEPROM();
        break;
    #endif

    #ifdef BattMonitor
      case 'N': // battery monitor
          batteryMonitorAlarmVoltage = readFloatSerial();
          batteryMonitorThrottleTarget = readFloatSerial();
          batteryMonitorGoingDownTime = readFloatSerial();
          setBatteryCellVoltageThreshold(batteryMonitorAlarmVoltage);
          writeEEPROM();
        break;
    #endif

    #ifdef CameraControl
      case 'P': //  read Camera values
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
          servoTXChannels = readFloatSerial();
          writeEEPROM();
          break;
    #endif      
      
    // receive receiver channel map
    case 'R':
      for (byte channel = 0; channel < LAST_CHANNEL; channel++)
      {
        receiverChannelMap[channel] = readFloatSerial();
      }
      writeEEPROM();
      break;

    // reset receiver channel map
    case 'S':
      for (byte channel = 0; channel < LAST_CHANNEL; channel++)
      {
        receiverChannelMap[channel] = channel;
      }
      writeEEPROM();
      break;

    #if defined (UseGPS)
      case 'U':
        isGpsEnabled = readFloatSerial();
        writeEEPROM();
        break;
      
      case 'V': // GPS
        readSerialPID(GPSROLL_PID_IDX);
        readSerialPID(GPSPITCH_PID_IDX);
        readSerialPID(GPSYAW_PID_IDX);
        writeEEPROM();
        break;
        
      case 'O': // define waypoints
        missionNbPoint = readIntegerSerial();
        waypoint[missionNbPoint].latitude = readIntegerSerial();
        waypoint[missionNbPoint].longitude = readIntegerSerial();
        waypoint[missionNbPoint].altitude = readIntegerSerial();
        writeEEPROM();
        break;
    #endif

    case 'W': // Write all user configurable values to EEPROM
      writeEEPROM(); // defined in DataStorage.h
      zeroIntegralError();
      break;

    case 'Y':
      flightConfigType = readFloatSerial();
      writeEEPROM();
      break;
      
    case 'Z':
      yawDirection = readFloatSerial();
      writeEEPROM();
      break;
      
    case 'J': 
      receiverTypeUsed = readFloatSerial();
      writeEEPROM();
      break;

    case '1': // Calibrate ESCS's by setting Throttle high on all channels
      validateCalibrateCommand(1);
      break;

    case '2': // Calibrate ESC's by setting Throttle low on all channels
      validateCalibrateCommand(2);
      break;

    case '5': // Send individual motor commands (motor, command)
      if (validateCalibrateCommand(5)) {
        for (byte motor = 0; motor < LASTMOTOR; motor++) {
          motorConfiguratorCommand[motor] = (int)readFloatSerial();
        }
      }
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
    if (vehicleState & MAG_DETECTED) {
      return trueNorthHeading;
    }
    else {
      return gyroHeading;
    }
  #else
    return gyroHeading;
  #endif
}

void sendSerialTelemetry() {
  
  switch (serialQueryType) {

    case 'a': // Send roll and pitch rate mode PID values
      PrintPID(RATE_XAXIS_PID_IDX);
      PrintPID(RATE_YAXIS_PID_IDX);
      PrintValueComma(rotationSpeedFactor);
      PrintValueComma(throttlePIDAdjustmentFactor);
      SERIAL_PRINTLN();
      serialQueryType = 'X';
      break;
  
    case 'b': // Send roll and pitch attitude mode PID values
      PrintPID(ATTITUDE_XAXIS_PID_IDX);
      PrintPID(ATTITUDE_YAXIS_PID_IDX);
      SERIAL_PRINTLN(0);
      serialQueryType = 'X';
      break;
  
    case 'c': // Send yaw PID values
      PrintPID(ZAXIS_PID_IDX);
      PrintValueComma(yawSpeedFactor);
      SERIAL_PRINTLN(0);
      serialQueryType = 'X';
      break;

    #if defined AltitudeHoldBaro
    case 'd': // Altitude Hold
        PrintPID(BARO_ALTITUDE_HOLD_PID_IDX);
        PrintValueComma(altitudeHoldBump);
        PrintValueComma(altitudeHoldMaxVelocitySpeed);
        PrintValueComma(baroSmoothFactor);
        PrintPID(ZDAMPENING_PID_IDX);
        SERIAL_PRINTLN();
        serialQueryType = 'X';
        break;
    #endif        
  
    case 'g': // Send transmitter calibration data
      for (byte channel = XAXIS; channel < LAST_CHANNEL; channel++) {
        Serial.print(receiverMinValue[channel]);
        Serial.print(',');
        Serial.print(receiverMaxValue[channel]);
        Serial.print(',');
      }
      SERIAL_PRINTLN();
      serialQueryType = 'X';
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
        PrintValueComma(zVelocity/100.0);
      #else
        PrintValueComma(0);
        PrintValueComma(0);
      #endif
      SERIAL_PRINTLN();
      break;

    #ifdef HeadingMagHold
      case 'j': // Send raw mag values
        PrintValueComma(rawMag[XAXIS]);
        PrintValueComma(rawMag[YAXIS]);
        SERIAL_PRINTLN(rawMag[ZAXIS]);
        break;
    #endif
    
    case 'l': // Send raw accel values
      evaluateMetersPerSec();    // reset sample data
      delay(2);
      measureCriticalSensors();  // call mesureAccelSum that give one raw sample in accelSample
      PrintValueComma((int)(accelSample[XAXIS]));
      PrintValueComma((int)(accelSample[YAXIS]));
      SERIAL_PRINTLN ((int)(accelSample[ZAXIS]));
      break;
  
    #ifdef BattMonitor
      case 'n': // battery monitor
        PrintValueComma(batteryMonitorAlarmVoltage);
        PrintValueComma(batteryMonitorThrottleTarget);
        PrintValueComma(batteryMonitorGoingDownTime);
        SERIAL_PRINTLN();
        serialQueryType = 'X';
        break;
    #endif      

    #if defined (UseGPS)
      case 'o': // send waypoints
        for (byte index = 0; index < MAX_WAYPOINTS; index++) {
          PrintValueComma(index);
          PrintValueComma(waypoint[index].latitude);
          PrintValueComma(waypoint[index].longitude);
          PrintValueComma(waypoint[index].altitude);
        }
        SERIAL_PRINTLN();
        serialQueryType = 'X';
        break;
    #endif        

    #ifdef CameraControl
      case 'p': // Send Camera values
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
        PrintValueComma(servoTXChannels);
        SERIAL_PRINTLN();
        serialQueryType = 'X';
        break;
    #endif
  
    case 'q': // Send Vehicle State Value
      SERIAL_PRINTLN(vehicleState);
      serialQueryType = 'X';
      break;
  
    case 's': // Send all flight data
      SERIAL_PRINT('S');
      PrintValueComma(motorArmed);
      PrintValueComma(kinematicsAngle[XAXIS]);
      PrintValueComma(kinematicsAngle[YAXIS]);
      PrintValueComma(getHeading());
      #if defined AltitudeHoldBaro
        PrintValueComma(estimatedAltitude);
        PrintValueComma(zVelocity/100.0);
        PrintValueComma((int)altitudeHoldState);
      #else
        PrintValueComma(0);
        PrintValueComma(0);
        PrintValueComma(0);
      #endif
  
      for (byte channel = 0; channel < 8; channel++) { // Configurator expects 8 values
        PrintValueComma(receiverCommand[receiverChannelMap[channel]]);
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
      
      #if defined (UseGPS)
        PrintValueComma(gpsData.state);
        PrintValueComma(gpsData.sats);
        PrintValueComma(gpsData.speed);
        PrintValueComma(gpsData.height);
        PrintValueComma(gpsData.course);
        PrintValueComma(gpsData.lat);
        PrintValueComma(gpsData.lon);
        PrintValueComma(gpsDistanceToDestination/100);
        PrintValueComma(degrees(angleToWaypoint));
      #else
        PrintValueComma(0);
        PrintValueComma(0);
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
  
    case 't': // Send raw transmitter values
      for (byte channel = 0; channel < LAST_CHANNEL; channel++) {
        PrintValueComma(receiverCommand[receiverChannelMap[channel]]);
      }
      SERIAL_PRINTLN();
      break;

    #if defined (UseGPS)
      case 'v': // Send GPS PIDs
        PrintPID(GPSROLL_PID_IDX);
        PrintPID(GPSPITCH_PID_IDX);
        PrintPID(GPSYAW_PID_IDX);
        serialQueryType = 'X';
        SERIAL_PRINTLN();
        serialQueryType = 'X';
        break;
    #endif

    #if defined (UseGPS)
      case 'y': // send GPS info
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
        SERIAL_PRINTLN();
        break;
    #endif    
      
    #if defined (BattMonitor)
      case '$': // send BatteryMonitor voltage/current readings
        PrintValueComma((float)batteryData[0].voltage/100.0); // voltage internally stored at 10mV:s
        #if defined (BM_EXTENDED)
          PrintValueComma((float)batteryData[0].current/100.0);
          PrintValueComma((float)batteryData[0].usedCapacity/1000.0);
        #else
          PrintDummyValues(2);
        #endif
        SERIAL_PRINTLN();
        break;
    #endif
      
    #if defined (UseAnalogRSSIReader) || defined (UseEzUHFRSSIReader) || defined (UseSBUSRSSIReader)
      case '%': // send RSSI
        SERIAL_PRINTLN(rssiRawValue);
        break;
    #endif
  
  
    case '!': // Send flight software version
      SERIAL_PRINTLN(SOFTWARE_VERSION, 1);
      serialQueryType = 'X';
      break;
  
    case '#': // Send configuration
      reportVehicleState();
      serialQueryType = 'X';
      break;
  
  #if defined(OSD) && defined(OSD_LOADFONT)
    case '&': // fontload
      if (OFF == motorArmed) {
        max7456LoadFont();
      }
      serialQueryType = 'X';
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

//void printVehicleState(const char *sensorName, unsigned long state, const char *message) {
//  
//  SERIAL_PRINT(sensorName);
//  SERIAL_PRINT(": ");
//  if (!(vehicleState & state)) {
//    SERIAL_PRINT("Not ");
//  }
//  SERIAL_PRINT(message);
//}

void reportVehicleState() {
  // Tell Configurator how many vehicle state values to expect
  SERIAL_PRINT(vehicleState);
  SERIAL_PRINT(";");
  delay(50);
  SERIAL_PRINT("Software Version: ");
  SERIAL_PRINT(SOFTWARE_VERSION);
  SERIAL_PRINT(";");
  delay(50);
  SERIAL_PRINT("Board Type: ");
  #if defined(AeroQuad_v18)
    SERIAL_PRINT("v1.8 and greater;");
  #elif defined(AeroQuad_Mini)
    SERIAL_PRINT("Mini;");
  #elif defined(MWCFlip15)
    SERIAL_PRINT("MWCFlip15;");
  #elif defined(AeroQuadMega_v2)
    SERIAL_PRINT("Mega v2;");
  #elif defined(AeroQuadMega_v21)
    SERIAL_PRINT("Mega v21;");
  #elif defined(MWCProEz30)
    SERIAL_PRINT("MWCProEz30;");    
  #elif defined(AeroQuadSTM32)
    SERIAL_PRINT(STM32_BOARD_TYPE);
    SERIAL_PRINT(";");
  #endif
  delay(50);

  SERIAL_PRINT("FlightConfig: ");
  SERIAL_PRINT(flightConfigType);
  SERIAL_PRINT(";");
  delay(50);
  
  SERIAL_PRINT("ReceiverType: ");
  SERIAL_PRINT(receiverTypeUsed);
  SERIAL_PRINT(";");
  delay(50);
  
  SERIAL_PRINT("ReceiverNbChannels: ");
  SERIAL_PRINT(LAST_CHANNEL);
  SERIAL_PRINT(";");
  delay(50);
  
  SERIAL_PRINT("Motors: ");
  SERIAL_PRINT(LASTMOTOR);
  SERIAL_PRINT(";");
  delay(50);
  
  SERIAL_PRINT("YawDirection: ");
  SERIAL_PRINT(yawDirection);
  SERIAL_PRINT(";");

  SERIAL_PRINTLN();
}

#endif // _AQ_SERIAL_COMM_
