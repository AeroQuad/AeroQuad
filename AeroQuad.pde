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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad below
   If you need additional assitance go to http://AeroQuad.com/forum
*****************************************************************************/

/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
#define XConfig
//#define plusConfig
//#define HEXACOAXIAL
//#define HEXARADIAL

// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// *******************************************************************************************************************************
#define UseArduPirateSuperStable // Enable the imported stable mode imported from ArduPirate (experimental, use at your own risk)
//#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
//#define AltitudeHold // Enables BMP085 Barometer (experimental, use at your own risk)
//#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable

// *******************************************************************************************************************************
// Camera Stabilization
// Servo output goes to D11(pitch), D12(roll), D13(yaw) on AeroQuad v1.8 shield
// If using v2.0 Shield place jumper between:
// D12 to D33 for roll, connect servo to SERVO1
// D11 to D34 for pitch, connect servo to SERVO2
// D13 to D35 for yaw, connect servo to SERVO3
// Please note that you will need to have battery connected to power on servos with v2.0 shield
// *******************************************************************************************************************************
//#define CameraControl

#include <EEPROM.h>
#include <Wire.h>
#include <I2C.h>
#include <AQMath.h>
#include <Axis.h>
#include <MotorsGlobalNames.h>
#include <ReceiverGlobalVariables.h>
#include "AeroQuad.h"
#include "PID.h"

/****************************************************************************
************************* Hardware Configuration ***************************
****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#include <AeroQuadV1.h>         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#include <AeroQuadV1_IDG.h>     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
#include <AeroQuadV18.h>        // Arduino 2009 with AeroQuad Shield v1.8
//#include <AeroQuadWii.h>        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#include <AeroQuadMegaV1.h>     // Arduino Mega with AeroQuad Shield v1.7 and below
//#include <AeroQuadMegaV2.h>     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii        // Arduino Mega with Wii Sensors and Aero/Quad Shield v2.x
//#define ArduCopter              // ArduPilot Mega (APM) with APM Sensor Board
//#define Multipilot              // Multipilot board with Lys344 and ADXL 610 Gyro (needs debug)
//#define MultipilotI2C           // Active Multipilot I2C and Mixertable (needs debug)
//#define AeroQuadMega_CHR6DM     // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM           // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer 
                                  // (just uncomment AltitudeHold for baro), and voltage divider

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/
 
//******************************************************
// Camera stabilization seem to not work on 328p cause
// of specific register used @see Kenny
// need to be fixed, CupOfTea, help please
//****************************************************** 

#ifdef ArduCopter
  #define LED_Red 35
  #define LED_Yellow 36
  #define LED_Green 37
  #define RELE_pin 47
  #define SW1_pin 41
  #define SW2_pin 40
  #define BUZZER 9
  #define PIANO_SW1 42
  #define PIANO_SW2 43
  
  #include <AQAPMADCSensorsAccessor.h>
  #include <IDG500_ADCGyroscope.h>
  IDG500_ADCGyroscope tempGyro;
  Gyroscope *_gyro = &tempGyro;
  #include <ADXL335_ADCAccelerometer.h>
  ADXL335_ADCAccelerometer tempAccel;
  Accelerometer *_accel = &tempAccel;
  #include <ReceiverForAPM.h>
  ReceiverForAPM tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <APMMotors.h>
  APMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngleDCM.h"
  FlightAngleDCM tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include <HMC5843Magnetometer.h>
    HMC5843Magnetometer tempCompass(_gyro);
    Compass *_compass = &tempCompass;
  #endif
  #ifdef AltitudeHold
    #include <BMP085BarometricSensor.h>
    BMP085BarometricSensor tempAltitude;
    AltitudeProvider *_altitudeProvider = &tempAltitude;
  #endif
  #ifdef BattMonitor
    #include <APMBatteryMonitor.h>
    APMBatteryMonitor tempBatteryMonitor;
    BatteryMonitor *_batteryMonitor = &tempBatteryMonitor;
  #endif
#endif

#ifdef AeroQuadMega_Wii
  #include <AQWiiSensorAccessor.h>
  AQWiiSensorAccessor _wiiSensorAccessor;
  #include <WiiAccelerometer.h>
  WiiAccelerometer tempAccel(_wiiSensorAccessor);
  Accelerometer *_accel = &tempAccel;
  #include <WiiGyroscope.h>
  WiiGyroscope tempGyro(_wiiSensorAccessor);
  Gyroscope *_gyro = &tempGyro;
  #include <ReceiverForMega.h>
  ReceiverForMega tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <PWMMotors.h>
  PWMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngleDCM.h"
  FlightAngleDCM tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include <AeroQuadCameraStabilizer.h>
    AeroQuadCameraStabilizer tempCamera;
    CameraStabilizer *_cameraStabilizer = &tempCamera;
  #endif
#endif

#ifdef AeroQuadMega_CHR6DM
  #include <CHR6DMSensorsAccessor.h>
  CHR6DM _chr6dm;
  #include <CHR6DMAccelerometer.h>
  CHR6DMAccelerometer tempAccel(_chr6dm);
  Accelerometer *_accel = &tempAccel;
  #include <CHR6DMGyroscope.h>
  CHR6DMGyroscope tempGyro(_chr6dm);
  Gyroscope *_gyro = &tempGyro;
  #include <ReceiverForMega.h>
  ReceiverForMega tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <PWMMotors.h>
  PWMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngleDCM.h"
  FlightAngleDCM tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #include <CHR6DMCompass.h>
  CHR6DMCompass tempCompass(_chr6dm);
  Compass *_compass = &tempCompass;
  #ifdef AltitudeHold
    #include <BMP085BarometricSensor.h>
    BMP085BarometricSensor tempAltitude;
    AltitudeProvider *_altitudeProvider = &tempAltitude;
  #endif
  #ifdef BattMonitor
    #include <APMBatteryMonitor.h>
    APMBatteryMonitor tempBatteryMonitor;
    BatteryMonitor *_batteryMonitor = &tempBatteryMonitor;
  #endif
  #ifdef CameraControl
    #include <AeroQuadCameraStabilizer.h>
    AeroQuadCameraStabilizer tempCamera;
    CameraStabilizer *_cameraStabilizer = &tempCamera;
  #endif
#endif

#ifdef APM_OP_CHR6DM
  #define LED_Red 35
  #define LED_Yellow 36
  #define LED_Green 37
  #define RELE_pin 47
  #define SW1_pin 41
  #define SW2_pin 40
  #define BUZZER 9
  #define PIANO_SW1 42
  #define PIANO_SW2 43
  #include <CHR6DMSensorsAccessor.h>
  CHR6DM _chr6dm;
  #include <CHR6DMAccelerometer.h>
  CHR6DMAccelerometer tempAccel(_chr6dm);
  Accelerometer *_accel = &tempAccel;
  #include <CHR6DMGyroscope.h>
  CHR6DMGyroscope tempGyro(_chr6dm);
  Gyroscope *_gyro = &tempGyro;
  #include <ReceiverForAPM.h>
  ReceiverForAPM tempReceiver;
  Receiver *_receiver = &tempReceiver;
  #include <APMMotors.h>
  APMMotors tempMotors;
  Motors *_motors = &tempMotors;
  #include "FlightAngleDCM.h"
//  FlightAngleCHR6DM tempFlightAngle;
  FlightAngleDCM tempFlightAngle;
  FlightAngleProcessor *_flightAngle = &tempFlightAngle;
  #include "CHR6DMCompass.h"
  CHR6DMCompass tempCompass(_chr6dm);
  Compass *_compass = &tempCompass;
  #ifdef AltitudeHold
    #include <BMP085BarometricSensor.h>
    BMP085BarometricSensor tempAltitude;
    AltitudeProvider *_altitudeProvider = &tempAltitude;
  #endif
  #ifdef BattMonitor
    #include "APMBatteryMonitor.h"
    APMBatteryMonitor tempBatteryMonitor;
    BatteryMonitor *_batteryMonitor = &tempBatteryMonitor;
  #endif
  #ifdef CameraControl
    #include <AeroQuadCameraStabilizer.h>
    AeroQuadCameraStabilizer tempCamera;
    CameraStabilizer *_cameraStabilizer = &tempCamera;
  #endif
#endif

//#ifdef Multipilot
//  MultipilotAccelerometer tempAccel;
//  Accelerometer *_accel = &tempAccel;
//  IDGIXZ500Gyroscope tempGyro;
//  Gyroscope *_gyro = &tempGyro;
//  ReceiverForMultipilot tempReceiver;
//  Receiver *_receiver = &tempReceiver;
//  PWMMotors tempMotors;
//  Motors *_motors = &tempMotors;
//  //#define PRINT_MIXERTABLE
//  //#define TELEMETRY_DEBUG
//  #include "FlightAngle.h"
//  FlightAngle_DCM tempFlightAngle;
//  FlightAngle *_flightAngle = &tempFlightAngle;
//#endif
//
//#ifdef MultipilotI2C  
//  MultipilotAccelerometer tempAccel;
//  Accelerometer *_accel = &tempAccel;
//  IDGIXZ500Gyroscope tempGyro;
//  Gyroscope *_gyro = &tempGyro;
//  ReceiverForMultipilot tempReceiver;
//  Receiver *_receiver = &tempReceiver;
//  Motors_I2C tempMotors;
//  Motors *_motors = &tempMotors;
//  //#define PRINT_MIXERTABLE
//  //#define TELEMETRY_DEBUG
//  #include "FlightAngle.h"
//  FlightAngle_DCM tempFlightAngle;
//  FlightAngle *_flightAngle = &tempFlightAngle;
//#endif



#ifdef XConfig
  void (*processFlightControl)() = &processFlightControlXMode;
#else
  void (*processFlightControl)() = &processFlightControlPlusMode;
#endif

#ifdef UseArduPirateSuperStable
  void (*processStableMode)() = &processArdupirateSuperStableMode;
#else
  void (*processStableMode)() = &processAeroQuadStableMode;
#endif

// Include this last as it contains objects from above declarations
#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  Serial.begin(BAUD);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    Serial1.begin(BAUD);
    PORTD = B00000100;
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);
  #endif
  #ifdef AeroQuadMega_v2
    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT);
    pinMode(34, INPUT);
    pinMode(35, INPUT);
  #endif
  #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);
  #endif
  
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM) || defined(ArduCopter)
    Wire.begin();
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2)
    // Recommendation from Mgros to increase I2C speed to 400kHz
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11262&viewfull=1#post11262
    TWBR = 12;
  #endif

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  // Configure motors
  _motors->initialize(); // defined in Motors.h

  // Setup receiver pins for pin change interrupts
  if (_receiverLoop == ON) 
  {
    _receiver->initialize(); // defined in Received.h
    initTransmitterFromEEPROM();
  }
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  
  _gyro->initialize(); // defined in Gyro.h
  _accel->initialize(); // defined in Accel.h
  initSensorsFromEEPROM();
  
  // Calibrate sensors
  zeroIntegralError();
  _levelAdjust[ROLL] = 0;
  _levelAdjust[PITCH] = 0;
  
  // Setup correct sensor orientation
  #ifdef AeroQuad_v1
    _gyro->invert(YAW);
  #endif
  #if defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii)
    _accel->invert(PITCH);
    _accel->invert(ZAXIS);
  #endif
  #ifdef Multipilot
    _accel->invert(PITCH);
    _gyro->invert(ROLL);
  #endif
  
  // Flight angle estimiation
  _flightAngle->initialize(); // defined in FlightAngle.h

  // Optional Sensors
  #ifdef HeadingMagHold
    _compass->initialize();
    _setHeading = _compass->getHeading();
  #endif
  #ifdef AltitudeHold
    _altitudeProvider->initialize();
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    _batteryMonitor->initialize();
  #endif
  
  // Camera stabilization setup
  #ifdef CameraControl
    _cameraStabilizer->initialize();
    _cameraStabilizer->setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    _cameraStabilizer->setCenterRoll(1500); // Need to figure out nice way to set center position
    _cameraStabilizer->setmCameraPitch(11.11);
    _cameraStabilizer->setCenterPitch(1300);
  #endif
  
  _previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
  _safetyCheck = 0;
}

// ************************************************************
// ******************** Main AeroQuad Loop ********************
// ************************************************************
void loop () {
  // Measure loop rate
  _currentTime = micros();
  _deltaTime = _currentTime - _previousTime;
  G_Dt = _deltaTime / 1000000.0;
  _previousTime = _currentTime;
  #ifdef DEBUG
    if (testSignal == LOW) testSignal = HIGH;
    else testSignal = LOW;
    digitalWrite(LEDPIN, testSignal);
  #endif
  
  // Measures sensor data and calculates attitude
  if (_sensorLoop == ON) {
    readSensors(); // defined in Sensors.pde
  } 

  // Combines external pilot commands and measured sensor data to generate motor commands
  if (_controlLoop == ON) {
    processFlightControl();
  } 
  
  // Reads external pilot commands and performs functions based on stick configuration
  if ((_receiverLoop == ON) && (_currentTime > _receiverTime)) {// 50Hz
    readPilotCommands(); // defined in FlightCommand.pde
    _receiverTime = _currentTime + RECEIVERLOOPTIME;
  }
  
  // Listen for configuration commands and reports telemetry
  if ((_telemetryLoop == ON) && (_currentTime > _telemetryTime)) { // 20Hz
    readSerialCommand(); // defined in SerialCom.pde
    sendSerialTelemetry(); // defined in SerialCom.pde
    _telemetryTime = _currentTime + TELEMETRYLOOPTIME;
  }

  #ifdef CameraControl // Experimental, not fully implemented yet
    if ((_cameraLoop == ON) && (_currentTime > _cameraTime)) // 50Hz
    { 
      _cameraStabilizer->setPitch(_flightAngle->getData(PITCH));
      _cameraStabilizer->setRoll(_flightAngle->getData(ROLL));
      _cameraStabilizer->setYaw(_flightAngle->getData(YAW));
      _cameraStabilizer->move();
      _cameraTime = _currentTime + CAMERALOOPTIME;
    }
  #endif
}
