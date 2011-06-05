/*
  AeroQuad v2.4.1 - June 2011
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
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software

//#define DEBUG_LOOP

//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v1_IDG     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
//#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8
//#define AeroQuad_Mini       // Arduino Pro Mini with AeroQuad Mini Shield V1.0
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuad_Paris_v3   // Define along with either AeroQuad_Wii to include specific changes for MultiWiiCopter Paris v3.0 board					
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with APM Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider

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
// You must define one of the next 3 attitude stabilization modes or the software will not build
// *******************************************************************************************************************************
#define HeadingMagHold // Enables HMC5843 Magnetometer, gets automatically selected if CHR6DM is defined
#define AltitudeHold // Enables BMP085 Barometer (experimental, use at your own risk)
#define BattMonitor //define your personal specs in BatteryMonitor.h! Full documentation with schematic there
//#define RateModeOnly // Use this if you only have a gyro sensor, this will disable any attitude modes.
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// You must define *only* one of the following 2 flightAngle calculations
// if you only want DCM, then don't define either of the below
// flightAngle recommendations: use FlightAngleARG if you do not have a magnetometer, use DCM if you have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define FlightAngleMARG // Experimental!  Fly at your own risk! Use this if you have a magnetometer installed and enabled HeadingMagHold above
//#define FlightAngleARG // Use this if you do not have a magnetometer installed
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable
//#define BinaryWrite // Enables fast binary transfer of flight data to Configurator
//#define BinaryWritePID // Enables fast binary transfer of attitude PID data
//#define OpenlogBinaryWrite // Enables fast binary transfer to serial1 and openlog hardware

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

// On screen display implementation using MAX7456 chip. See OSD.h for more info and configuration.
//#define MAX7456_OSD

/****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************/
// Checks to make sure we have the right combinations defined
#if defined(FlightAngleMARG) && !defined(HeadingMagHold)
#undef FlightAngleMARG
#endif
#if defined(HeadingMagHold) && defined(FlightAngleMARG) && defined(FlightAngleARG)
#undef FlightAngleARG
#endif
#if defined(MAX7456_OSD) && !defined(AeroQuadMega_v2) && !defined(AeroQuadMega_Wii)
#undef MAX7456_OSD
#endif

#include <EEPROM.h>
#include <Wire.h>
#include "AeroQuad.h"
#include "I2C.h"
#include "PID.h"
#include "AQMath.h"
#include "Receiver.h"
#include "DataAcquisition.h"
#include "Accel.h"
#include "Gyro.h"
#include "Motors.h"

// Create objects defined from Configuration Section above
#ifdef AeroQuad_v1
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_v1_IDG
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_v18
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuad_Mini
  Accel_AeroQuadMini accel;
  Gyro_AeroQuadMega_v2 gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_v1
  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  Receiver_AeroQuadMega receiver;
  Accel_AeroQuad_v1 accel;
  Gyro_AeroQuad_v1 gyro;
  Motors_PWM motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_v2
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  //Motors_AeroQuadI2C motors; // Use for I2C based ESC's
  Accel_AeroQuadMega_v2 accel;
  Gyro_AeroQuadMega_v2 gyro;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

#ifdef ArduCopter
  Gyro_ArduCopter gyro;
  Accel_ArduCopter accel;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
#endif

#ifdef AeroQuad_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuad receiver;
  Motors_PWMtimer motors;
  #include "FlightAngle.h"
//  FlightAngle_CompFilter tempFlightAngle;
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
  #ifdef MAX7456_OSD
    #include "OSD.h"
    OSD osd;
  #endif
#endif

#ifdef AeroQuadMega_Wii
  Accel_Wii accel;
  Gyro_Wii gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWMtimer motors;
  #include "FlightAngle.h"
  #ifdef FlightAngleARG
    FlightAngle_ARG tempFlightAngle;
  #elif defined FlightAngleMARG
    FlightAngle_MARG tempFlightAngle;
  #else
    FlightAngle_DCM tempFlightAngle;
  #endif
  FlightAngle *flightAngle = &tempFlightAngle;
  #ifdef HeadingMagHold
    #include "Compass.h"
    Magnetometer_HMC5843 compass;
  #endif
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_AeroQuad batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef AeroQuadMega_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_AeroQuadMega receiver;
  Motors_PWM motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM tempFlightAngle;
  FlightAngle *flightAngle = &tempFlightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef APM_OP_CHR6DM
  Accel_CHR6DM accel;
  Gyro_CHR6DM gyro;
  Receiver_ArduCopter receiver;
  Motors_ArduCopter motors;
  #include "FlightAngle.h"
  FlightAngle_CHR6DM tempFlightAngle;
  FlightAngle *flightAngle = &tempFlightAngle;
  #include "Compass.h"
  Compass_CHR6DM compass;
  #ifdef AltitudeHold
    #include "Altitude.h"
    Altitude_AeroQuad_v2 altitude;
  #endif
  #ifdef BattMonitor
    #include "BatteryMonitor.h"
    BatteryMonitor_APM batteryMonitor;
  #endif
  #ifdef CameraControl
    #include "Camera.h"
    Camera_AeroQuad camera;
  #endif
#endif

#ifdef XConfig
  void (*processFlightControl)() = &processFlightControlXMode;
#endif
#ifdef plusConfig
  void (*processFlightControl)() = &processFlightControlPlusMode;
#endif

// Include this last as it contains objects from above declarations
#include "DataStorage.h"

// ************************************************************
// ********************** Setup AeroQuad **********************
// ************************************************************
void setup() {
  SERIAL_BEGIN(BAUD);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);

#ifdef DEBUG_LOOP
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  digitalWrite(12, LOW);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
#endif    

  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    Serial1.begin(BAUD);
    PORTD = B00000100;
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini)
    pinMode(LED2PIN, OUTPUT);
    digitalWrite(LED2PIN, LOW);
    pinMode(LED3PIN, OUTPUT);
    digitalWrite(LED3PIN, LOW);
  #endif
  #ifdef AeroQuadMega_v2
    // pins set to INPUT for camera stabilization so won't interfere with new camera class
    pinMode(33, INPUT); // disable SERVO 1, jumper D12 for roll
    pinMode(34, INPUT); // disable SERVO 2, jumper D11 for pitch
    pinMode(35, INPUT); // disable SERVO 3, jumper D13 for yaw
    pinMode(43, OUTPUT); // LED 1
    pinMode(44, OUTPUT); // LED 2
    pinMode(45, OUTPUT); // LED 3
    pinMode(46, OUTPUT); // LED 4
    digitalWrite(43, HIGH); // LED 1 on
    digitalWrite(44, HIGH); // LED 2 on
    digitalWrite(45, HIGH); // LED 3 on
    digitalWrite(46, HIGH); // LED 4 on  
  #endif
  #if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);
  #endif
  
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini) || defined(AeroQuad_Wii) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM) || defined(ArduCopter)
    Wire.begin();
  #endif
  #if defined(AeroQuad_v18) || defined(AeroQuadMega_v2) || defined(AeroQuad_Mini)
    // Recommendation from Mgros to increase I2C speed to 400kHz
    // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11262&viewfull=1#post11262
    TWBR = 12;
  #endif

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  
  // Configure motors
  motors.initialize(); // defined in Motors.h

  // Setup receiver pins for pin change interrupts
  if (receiverLoop == ON) receiver.initialize(); // defined in Received.h
       
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  gyro.initialize(); // defined in Gyro.h
  accel.initialize(); // defined in Accel.h
  
  // Calibrate sensors
  gyro.autoZero(); // defined in Gyro.h
  zeroIntegralError();
 // levelAdjust[ROLL] = 0;
 // levelAdjust[PITCH] = 0;
  
  // Flight angle estimation
  #ifdef HeadingMagHold
    compass.initialize();
    //setHeading = compass.getHeading();
    flightAngle->initialize(compass.getHdgXY(XAXIS), compass.getHdgXY(YAXIS));
  #else
    flightAngle->initialize(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[LEVELROLL].windupGuard = 0.375;
  PID[LEVELPITCH].windupGuard = 0.375;

  // Optional Sensors
  #ifdef AltitudeHold
    altitude.initialize();
  #endif
  
  // Battery Monitor
  #ifdef BattMonitor
    batteryMonitor.initialize();
  #endif
  
  // Camera stabilization setup
  #ifdef CameraControl
    camera.initialize();
    camera.setmCameraRoll(11.11); // Need to figure out nice way to reverse servos
    camera.setCenterRoll(1500); // Need to figure out nice way to set center position
    camera.setmCameraPitch(11.11);
    camera.setCenterPitch(1300);
  #endif
  
  //initialising OSD
  #if defined(MAX7456_OSD)
    osd.initialize();
  #endif

  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial1;
      binaryPort->begin(115200);
      delay(1000);
    #else
     binaryPort = &Serial;
    #endif 
  #endif
  
  // AKA use a new low pass filter called a Lag Filter uncomment only if using DCM LAG filters
  //  setupFilters(accel.accelOneG);

  previousTime = micros();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

/*******************************************************************
  // tasks (microseconds of interval)
  ReadGyro        readGyro      (   5000); // 200hz
  ReadAccel       readAccel     (   5000); // 200hz
  RunDCM          runDCM        (  10000); // 100hz
  FlightControls  flightControls(  10000); // 100hz
  ReadReceiver    readReceiver  (  20000); //  50hz
  ReadBaro        readBaro      (  40000); //  25hz
  ReadCompass     readCompass   ( 100000); //  10Hz
  ProcessTelem    processTelem  ( 100000); //  10Hz
  ReadBattery     readBattery   ( 100000); //  10Hz
  
  Task *tasks[] = {&readGyro, &readAccel, &runDCM, &flightControls,   \
                   &readReceiver, &readBaro, &readCompass,            \
                   &processTelem, &readBattery};
                   
  TaskScheduler sched(tasks, NUM_TASKS(tasks));
  
  sched.run();
*******************************************************************/
void loop () {
  currentTime = micros();
  deltaTime = currentTime - previousTime;
  
  // Main scheduler loop set for 100hz
  if (deltaTime >= 10000) {

    #ifdef DEBUG_LOOP
      testSignal ^= HIGH;
      digitalWrite(LEDPIN, testSignal);
    #endif

    frameCounter++;
    
    // ================================================================
    // 100hz task loop
    // ================================================================
    if (frameCounter %   1 == 0) {  //  100 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(11, HIGH);
      #endif
      
      G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      hundredHZpreviousTime = currentTime;
      
      if (sensorLoop == ON) {
        // measure critical sensors
        gyro.measure();
        accel.measure();
        
        // ****************** Calculate Absolute Angle *****************
        #if defined HeadingMagHold && defined FlightAngleMARG
          flightAngle->calculate(gyro.getData(ROLL),                       \
                                 gyro.getData(PITCH),                      \
                                 gyro.getData(YAW),                        \
                                 accel.getData(XAXIS),                     \
                                 accel.getData(YAXIS),                     \
                                 accel.getData(ZAXIS),                     \
                                 compass.getRawData(XAXIS),                \
                                 compass.getRawData(YAXIS),                \
                                 compass.getRawData(ZAXIS));
        #endif
      
        #if defined HeadingMagHold && defined FlightAngleARG
          flightAngle->calculate(gyro.getData(ROLL),                       \
                                 gyro.getData(PITCH),                      \
                                 gyro.getData(YAW),                        \
                                 accel.getData(XAXIS),                     \
                                 accel.getData(YAXIS),                     \
                                 accel.getData(ZAXIS),                     \
                                 0.0,                                      \
                                 0.0,                                      \
                                 0.0);
        #endif

        #if !defined HeadingMagHold && defined FlightAngleARG
          flightAngle->calculate(gyro.getData(ROLL),                       \
                                 gyro.getData(PITCH),                      \
                                 gyro.getData(YAW),                        \
                                 accel.getData(XAXIS),                     \
                                 accel.getData(YAXIS),                     \
                                 accel.getData(ZAXIS),                     \
                                 0.0,                                      \
                                 0.0,                                      \
                                 0.0);
        #endif
      
        #if defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
          flightAngle->calculate(gyro.getData(ROLL),                       \
                                 gyro.getData(PITCH),                      \
                                 gyro.getData(YAW),                        \
                                 accel.getData(XAXIS),                     \
                                 accel.getData(YAXIS),                     \
                                 accel.getData(ZAXIS),                     \
                                 accel.getOneG(),                          \
                                 compass.getHdgXY(XAXIS),                  \
                                 compass.getHdgXY(YAXIS));
        #endif
        
        #if !defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
          flightAngle->calculate(gyro.getData(ROLL),  \
                                 gyro.getData(PITCH),                      \
                                 gyro.getData(YAW),                        \
                                 accel.getData(XAXIS),                     \
                                 accel.getData(YAXIS),                     \
                                 accel.getData(ZAXIS),                     \
                                 accel.getOneG(),                          \
                                 0.0,                                      \
                                 0.0);
        #endif
      }
      
      // Combines external pilot commands and measured sensor data to generate motor commands
      if (controlLoop == ON) {
        processFlightControl();
      } 

      #ifdef BinaryWrite
        if (fastTransfer == ON) {
          // write out fastTelemetry to Configurator or openLog
          fastTelemetry();
        }
      #endif      
      
      #ifdef DEBUG_LOOP
        digitalWrite(11, LOW);
      #endif
    }

    // ================================================================
    // 50hz task loop
    // ================================================================
    if (frameCounter %   2 == 0) {  //  50 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(10, HIGH);
      #endif
      
      G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;
      
      // Reads external pilot commands and performs functions based on stick configuration
      if (receiverLoop == ON) { 
        readPilotCommands(); // defined in FlightCommand.pde
      }

      #if defined(CameraControl)
        camera.setPitch(degrees(flightAngle->getData(PITCH)));
        camera.setRoll(degrees(flightAngle->getData(ROLL)));
        camera.setYaw(degrees(flightAngle->getData(YAW)));
        camera.move();
      #endif 

      #ifdef DEBUG_LOOP
        digitalWrite(10, LOW);
      #endif
    }

    // ================================================================
    // 25hz task loop
    // ================================================================
    if (frameCounter %   4 == 0) {  //  25 Hz tasks
      #ifdef DEBUG_LOOP    
        digitalWrite(9, HIGH);
      #endif
      
      G_Dt = (currentTime - twentyFiveHZpreviousTime) / 1000000.0;
      twentyFiveHZpreviousTime = currentTime;
      
      if (sensorLoop == ON) {
        #if defined(AltitudeHold)
          altitude.measure(); // defined in altitude.h
        #endif
      }
      
      #ifdef DEBUG_LOOP
        digitalWrite(9, LOW);
      #endif
    }
    
    // ================================================================
    // 10hz task loop
    // ================================================================
    if (frameCounter %  10 == 0) {  //   10 Hz tasks
      #ifdef DEBUG_LOOP
        digitalWrite(8, HIGH);
      #endif
      
      G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
      tenHZpreviousTime = currentTime;

      if (sensorLoop == ON) {
        #if defined(HeadingMagHold)
          compass.measure(flightAngle->getData(ROLL), flightAngle->getData(PITCH)); // defined in compass.h
        #endif
        #if defined(BattMonitor)
          batteryMonitor.measure(armed);
        #endif
      }
      // Listen for configuration commands and reports telemetry
      if (telemetryLoop == ON) {
        readSerialCommand(); // defined in SerialCom.pde
        sendSerialTelemetry(); // defined in SerialCom.pde
      }
      
      #ifdef MAX7456_OSD
        osd.update();
      #endif

      #ifdef DEBUG_LOOP
        digitalWrite(8, LOW);
      #endif      
	}

    previousTime = currentTime;
  }
  if (frameCounter >= 100) 
      frameCounter = 0;
}


