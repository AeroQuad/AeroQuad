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

/****************************************************************************
   Before flight, select the different user options for your AeroQuad by
   editing UserConfiguration.h.

   If you need additional assistance go to http://www.aeroquad.com/forum.php
   or talk to us live on IRC #aeroquad
*****************************************************************************/

#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//
// Define Security Checks
//

#if defined(UseGPSNMEA) || defined(UseGPSUBLOX) || defined(UseGPSMTK) || defined(UseGPS406)
 #define UseGPS
#endif 

#if defined(UseGPSNavigator) && !defined(AltitudeHoldBaro)
  #error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#if defined(AutoLanding) && (!defined(AltitudeHoldBaro) || !defined(AltitudeHoldRangeFinder))
  #error "AutoLanding NEED AltitudeHoldBaro and AltitudeHoldRangeFinder defined"
#endif

#if defined(ReceiverSBUS) && defined(SlowTelemetry)
  #error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
#endif

#if defined (CameraTXControl) && !defined (CameraControl)
  #error "CameraTXControl need to have CameraControl defined"
#endif 

#if defined (AeroQuadMega_v2) || defined (AeroQuadMega_v21) || defined (MWCProEz30) || defined (AeroQuadSTM32)
  #define HeadingMagHold		
  #define AltitudeHoldBaro		
#endif

#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>

#include <FourtOrderFilter.h>
#ifdef BattMonitor
  #include <BatteryMonitorTypes.h>
#endif

#include <vector3.h>

//********************************************************
//********************************************************
//********* PLATFORM SPECIFIC SECTION ********************
//********************************************************
//********************************************************
#ifdef AeroQuad_v18
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_BMA180.h>

  // Receiver declaration
  #include <Receiver_328p.h>

  // Motor declaration
  #include <Motors_328p.h>
  
  #include <FlightConfig328p.h>

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15, 0.9, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_v18 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

    Wire.begin();
    TWBR = 12;
    
    switch (flightConfigType) 
    {
      case HEX_Y6 :
      case HEX_PLUS :
      case HEX_X :
        LASTMOTOR = 6;
        break;
      default:
        LASTMOTOR = 4;
    }
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0047340002;
    accelScaleFactor[YAXIS] = -0.0046519994;
    accelScaleFactor[ZAXIS] = -0.0046799998;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }

#endif

#ifdef AeroQuad_Mini
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #define ITG3200_ADDRESS_ALTERNATE
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345.h>

  // Receiver declaration
  #include <Receiver_328p.h>

  #include <Motors_328p.h>
  
  #include <FlightConfig328p.h>

  // Battery Monitor declaration
  #ifdef BattMonitor
    #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.53, BM_NOPIN, 0, 0)
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  // unsupported in mini
  #undef HeadingMagHold
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_Mini specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

    Wire.begin();
    TWBR = 12;
    
    switch (flightConfigType) 
    {
      case HEX_Y6 :
      case HEX_PLUS :
      case HEX_X :
        LASTMOTOR = 6;
        break;
      default:
        LASTMOTOR = 4;
    }
  }

  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0371299982;
    accelScaleFactor[YAXIS] = -0.0374319982;
    accelScaleFactor[ZAXIS] = -0.0385979986;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }
#endif

#ifdef MWCFlip15
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  #define MPU6000_I2C
  #include <Platform_MPU6000.h>
  // Gyroscope declaration
  #include <Gyroscope_MPU6000.h>
  // Accelerometer declaration
  #include <Accelerometer_MPU6000.h>

  // Receiver declaration
  #include <Receiver_328p.h>

  // Motor declaration
  #include <Motors_328p.h>
  
  #include <FlightConfig328p.h>

  // unsupported in mini
  #undef HeadingMagHold
  #undef BattMonitor
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef CameraControl
  #undef OSD
  #undef UseGPS
  #undef UseGPSNavigator

  /**
   * Put AeroQuad_Mini specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

    Wire.begin();
    TWBR = 12;
    
    switch (flightConfigType) 
    {
      case HEX_Y6 :
      case HEX_PLUS :
      case HEX_X :
        LASTMOTOR = 6;
        break;
      default:
        LASTMOTOR = 4;
    }
    
    initializeMPU6000Sensors();
  }

  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0011980000;
    accelScaleFactor[YAXIS] = -0.0012020000;
    accelScaleFactor[ZAXIS] = -0.0011750000;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    readMPU6000Sensors();
    measureGyroSum();
    measureAccelSum();
  }

#endif

#ifdef AeroQuadMega_v2
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_BMA180.h>

  // Receiver declaration
  #include <Receiver_MEGA.h>

  // Motor declaration
  #include <Motors_MEGA.h>
  
  #include <FlightConfigMEGA.h>

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #include <Compass.h>
//    #define SPARKFUN_5883L_BOB
    #define HMC5843
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro    
    #define BMP085 
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    #ifdef POWERED_BY_VIN
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
    #else
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
    #endif
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif

  /**
   * Put AeroQuadMega_v2 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

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

    Wire.begin();
    TWBR = 12;
    
    switch (flightConfigType) 
    {
      case OCTO_X :
      case OCTO_PLUS :
      case OCTO_X8 :
        LASTMOTOR = 8;
        break;
      case HEX_Y6 :
      case HEX_PLUS :
      case HEX_X :
        LASTMOTOR = 6;
        break;
      default:
        LASTMOTOR = 4;
    }
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0046449995;
    accelScaleFactor[YAXIS] = -0.0047950000;
    accelScaleFactor[ZAXIS] = -0.0047549996;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 60.000000;
      magBias[YAXIS]  = -39.000000;
      magBias[ZAXIS]  = -7.500000;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }
#endif

#ifdef AeroQuadMega_v21
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  #include <Device_I2C.h>
  // Gyroscope declaration
  #define ITG3200_ADDRESS_ALTERNATE
  #include <Gyroscope_ITG3200_9DOF.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345_9DOF.h>
  
  // Receiver declaration
  #include <Receiver_MEGA.h>
  
  // Motor declaration
  #include <Motors_MEGA.h>
  
  #include <FlightConfigMEGA.h>

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #include <Compass.h>
    #define SPARKFUN_9DOF_5883L
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif


  // Battery Monitor declaration
  #ifdef BattMonitor
    #ifdef POWERED_BY_VIN
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
    #else
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
    #endif
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif


  /**
   * Put AeroQuadMega_v21 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

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

    Wire.begin();
    TWBR = 12;
    
    switch (flightConfigType) 
    {
      case OCTO_X :
      case OCTO_PLUS :
      case OCTO_X8 :
        LASTMOTOR = 8;
        break;
      case HEX_Y6 :
      case HEX_PLUS :
      case HEX_X :
        LASTMOTOR = 6;
        break;
      default:
        LASTMOTOR = 4;
    }
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0365570020;
    accelScaleFactor[YAXIS] = 0.0363000011;
    accelScaleFactor[ZAXIS] = -0.0384629964;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 1.500000;
      magBias[YAXIS]  = 205.500000;
      magBias[ZAXIS]  = -33.000000;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
    measureAccelSum();
  }
#endif


#ifdef MWCProEz30
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31
  
  #include <Device_I2C.h>
  
  #ifdef HeadingMagHold
    #include <Compass.h>
    #define HMC5883L
  #endif

  #define MPU6000_I2C
  #include <Platform_MPU6000.h>
  // Gyroscope declaration
  #include <Gyroscope_MPU6000.h>
  // Accelerometer declaration
  #include <Accelerometer_MPU6000.h>
  
  // Receiver declaration
  #define PPM_ON_THROTTLE
  #include <Receiver_MEGA.h>

  // Motor declaration
  #include <Motors_MEGA.h>

  #include <FlightConfigMEGA.h>

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define MS5611
    #define USE_MS5611_ALTERNATE_ADDRESS
    #define USE_Z_DAMPENING
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif


  // Battery Monitor declaration
  #ifdef BattMonitor
    #ifdef POWERED_BY_VIN
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0, BM_NOPIN, 0, 0) // v2 shield powered via VIN (no diode)
    #else
      #define BattDefaultConfig DEFINE_BATTERY(0, 0, 15.0, 0.82, BM_NOPIN, 0, 0) // v2 shield powered via power jack
    #endif
  #else
    #undef BattMonitorAutoDescent
    #undef POWERED_BY_VIN        
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  
  
  #ifndef UseGPS
    #undef UseGPSNavigator
  #endif


  /**
   * Put AeroQuadMega_v21 specific initialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

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

    Wire.begin();
    TWBR = 12;
    
    switch (flightConfigType) 
    {
      case OCTO_X :
      case OCTO_PLUS :
      case OCTO_X8 :
        LASTMOTOR = 8;
        break;
      case HEX_Y6 :
      case HEX_PLUS :
      case HEX_X :
        LASTMOTOR = 6;
        break;
      default:
        LASTMOTOR = 4;
    }
    
    initializeMPU6000Sensors();
  }
  
  // called when eeprom is initialized
  void initializePlatformSpecificAccelCalibration() {
    // Kenny default value, a real accel calibration is strongly recommended
    accelScaleFactor[XAXIS] = 0.0011980000;
    accelScaleFactor[YAXIS] = -0.0012020000;
    accelScaleFactor[ZAXIS] = -0.0011750000;
    #ifdef HeadingMagHold
      magBias[XAXIS]  = 1.500000;
      magBias[YAXIS]  = 205.500000;
      magBias[ZAXIS]  = -33.000000;
    #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    readMPU6000Sensors();
    measureGyroSum();
    measureAccelSum();
  }

#endif

//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

#ifdef AeroQuadSTM32
  #include "AeroQuad_STM32.h"
#endif

// default to 10bit ADC (AVR)
#ifndef ADC_NUMBER_OF_BITS
#define ADC_NUMBER_OF_BITS 10
#endif

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined(HeadingMagHold)
  #include "Kinematics_MARG.h"
#else
  #include "Kinematics_ARG.h"
#endif



//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(UseAnalogRSSIReader) 
  #include <AnalogRSSIReader.h>
#elif defined(UseEzUHFRSSIReader)
  #include <EzUHFRSSIReader.h>
#elif defined(UseSBUSRSSIReader)
  #include <SBUSRSSIReader.h>
#endif



//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined(triConfig)
  #if defined (MOTOR_STM32)
    #define MOTORS_STM32_TRI
    #include <Motors_STM32.h>    
  #else
    #include <Motors_Tri.h>
  #endif
#elif defined(MOTOR_PWM)
  #include <Motors_PWM.h>
#elif defined(MOTOR_PWM_Timer)
  #include <Motors_PWM_Timer.h>
#elif defined(MOTOR_APM)
  #include <Motors_APM.h>
#elif defined(MOTOR_I2C)
  #include <Motors_I2C.h>
#elif defined(MOTOR_STM32)
  #include <Motors_STM32.h>    
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843)
//  #include <HeadingFusionProcessorMARG.h>
  #include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L)
//  #include <HeadingFusionProcessorMARG.h>
  #include <Magnetometer_HMC5883L.h>
#elif defined(COMPASS_CHR6DM)
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
  #include <BarometricSensor_BMP085.h>
#elif defined(MS5611)
 #include <BarometricSensor_MS5611.h>
 #include <VelocityProcessor.h>
#endif
#if defined(XLMAXSONAR)
  #include <MaxSonarRangeFinder.h>
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor
  #include <BatteryMonitor.h>
  #ifndef BattCustomConfig
    #define BattCustomConfig BattDefaultConfig
  #endif
  struct BatteryData batteryData[] = {BattCustomConfig};
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#if defined(CameraControl_STM32)
  #include <CameraStabilizer_STM32.h>
#elif defined(CameraControl)
  #include <CameraStabilizer_Aeroquad.h>
#endif

#if defined (CameraTXControl)
  #include <CameraStabilizer_TXControl.h>
#endif

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS)
  #if !defined(HeadingMagHold)
    #error We need the magnetometer to use the GPS
  #endif 
  #include <GpsAdapter.h>
  #include "GpsNavigator.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
  #include <Device_SPI.h>
  #include "OSDDisplayController.h"
  #include "MAX7456.h"
#endif

#if defined(SERIAL_LCD)
  #include "SerialLCD.h"
#endif

#ifdef OSD_SYSTEM_MENU
  #if !defined(MAX7456_OSD) && !defined(SERIAL_LCD)
    #error "Menu cannot be used without OSD or LCD"
  #endif
  #include "OSDMenu.h"
#endif


//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined(AeroQuadSTM32) && defined (SERIAL_USES_USB)
  #define SERIAL_PORT SerialUSB
  #undef BAUD
  #define BAUD
#elif defined (USE_WIRELESS_COMMUNICATION)
  #define SERIAL_PORT Serial1
#else
  #define SERIAL_PORT Serial
#endif

// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
  #include "LedStatusProcessor.h"
#endif  

#if defined(MavLink)
  #include "MavLink.h"
#else
  #include "SerialCom.h"
#endif



/*******************************************************************
 * Main setup function, called one time at bootup
 * initialize all system and sub system of the
 * Aeroquad
 ******************************************************************/
void setup() {
  SERIAL_BEGIN(BAUD);
  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);

  initCommunication();
  readEEPROM(); 
  
  boolean firstTimeBoot = false;
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all EEPROM
    initializeEEPROM();
    writeEEPROM();
    firstTimeBoot = true;
  }
  
  initPlatform();
  
  initializeMotors(LASTMOTOR);

  (*initializeReceiver[receiverTypeUsed])();
  
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initializeGyro(); // defined in Gyro.h
  while (!calibrateGyro()); // this make sure the craft is still befor to continue init process
  initializeAccel(); // defined in Accel.h
  if (firstTimeBoot) {
    computeAccelBias();
    writeEEPROM();
  }
  setupFourthOrder();
  initSensorsZeroFromEEPROM();
  
  #ifdef HeadingMagHold
    initializeMagnetometer();
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeKinematics(0.0, 0.0, -accelOneG, measuredMag[XAXIS], measuredMag[XAXIS], measuredMag[XAXIS]);
  #else    
    initializeKinematics();
  #endif
  
  // Optional Sensors
  #ifdef AltitudeHoldBaro
    initializeBaro();
    vehicleState |= ALTITUDEHOLD_ENABLED;
  #endif
  #ifdef AltitudeHoldRangeFinder
    inititalizeRangeFinders();
    vehicleState |= RANGE_ENABLED;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].P = PID[BARO_ALTITUDE_HOLD_PID_IDX].P*2;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].I = PID[BARO_ALTITUDE_HOLD_PID_IDX].I;
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].D = PID[BARO_ALTITUDE_HOLD_PID_IDX].D;
  #endif
  
  #ifdef BattMonitor
    initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
    vehicleState |= BATTMONITOR_ENABLED;
  #endif
  
  #if defined(CameraControl)
    initializeCameraStabilization();
    vehicleState |= CAMERASTABLE_ENABLED;
  #endif

  #if defined(MAX7456_OSD)
    initializeSPI();
    initializeOSD();
  #endif
  
  #if defined(SERIAL_LCD)
    InitSerialLCD();
  #endif

  #if defined(UseGPS)
    initializeGps();
  #endif 

  #ifdef SlowTelemetry
     initSlowTelemetry();
  #endif

  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
  safetyCheck = 0;
}


/*******************************************************************
 * 100Hz task
 ******************************************************************/
void process100HzTask() {
  
  G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
  hundredHZpreviousTime = currentTime;
  
  evaluateGyroRate();
  evaluateMetersPerSec();

  for (int axis = XAXIS; axis <= ZAXIS; axis++) {
    filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
  }
   
  #if defined (HeadingMagHold) 
    calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], measuredMag[XAXIS], measuredMag[XAXIS], measuredMag[XAXIS], G_Dt);
    magDataUpdate = false;
  #else
    calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);
  #endif

  #if defined (AltitudeHoldBaro)
    if (vehicleState & BARO_DETECTED)
    {
      measureBaro();
//    measureBaroSum();
      #if defined USE_Z_DAMPENING
        float filteredZAccel = -(meterPerSecSec[XAXIS] * kinematicCorrectedAccel[XAXIS]
                                   + meterPerSecSec[YAXIS] * kinematicCorrectedAccel[YAXIS]
                                   + meterPerSecSec[ZAXIS] * kinematicCorrectedAccel[ZAXIS]);
        computeVelocity(filteredZAccel, G_Dt);
      #endif
    
      if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  //  50 Hz tasks
//      evaluateBaroAltitude();
//        measureBaro();
  
        #if defined USE_Z_DAMPENING      
          computeVelocityErrorFromBaroAltitude(getBaroAltitude());
          zVelocity = computedZVelocity;
  
          float estimatedBaroAltitude = filterSmooth(getBaroAltitude(), previousBaroAltitude, 0.01);
          estimatedBaroAltitude = (estimatedBaroAltitude) + ((zVelocity / 100.0) / 50.0);
          estimatedAltitude = filterSmooth(estimatedBaroAltitude, estimatedAltitude, 0.05);
          previousBaroAltitude = getBaroAltitude();
        #else
          estimatedAltitude = getBaroAltitude();
        #endif 
      }
    }
  #endif
        
  processFlightControl();
  
  #if defined(UseGPS)
    updateGps();
  #endif      
  
  #if defined(CameraControl)
    moveCamera(kinematicsAngle[YAXIS],kinematicsAngle[XAXIS],kinematicsAngle[ZAXIS]);
    #if defined CameraTXControl
      processCameraTXControl();
    #endif
  #endif       

}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
void process50HzTask() {
  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;

  // Reads external pilot commands and performs functions based on stick configuration
  readPilotCommands(); 
  
  #if defined(UseAnalogRSSIReader) || defined(UseEzUHFRSSIReader) || defined(UseSBUSRSSIReader)
    readRSSI();
  #endif

  #ifdef AltitudeHoldRangeFinder
    updateRangeFinders();
  #endif

  #if defined(UseGPS)
    if (haveAGpsLock() && !isHomeBaseInitialized()) {
      initHomeBase();
    }
  #endif      
}

/*******************************************************************
 * 10Hz task
 ******************************************************************/
void process10HzTask1() {
  
  #if defined(HeadingMagHold)
  
    G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
    tenHZpreviousTime = currentTime;
    if (vehicleState & MAG_DETECTED) {
      measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
      magDataUpdate = true;
    }
  #endif
}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
void process10HzTask2() {
  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;
  
  #if defined(BattMonitor)
    measureBatteryVoltage(G_Dt*1000.0);
  #endif

  // Listen for configuration commands and reports telemetry
  readSerialCommand();
  sendSerialTelemetry();
}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
void process10HzTask3() {
    G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
    lowPriorityTenHZpreviousTime2 = currentTime;

    #ifdef OSD_SYSTEM_MENU
      updateOSDMenu();
    #endif

    #ifdef MAX7456_OSD
      updateOSD();
    #endif
    
    #if defined(UseGPS) || defined(BattMonitor)
      processLedStatus();
    #endif
    
    #ifdef SlowTelemetry
      updateSlowTelemetry10Hz();
    #endif
}

/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {
  #ifdef MavLink
    G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
    oneHZpreviousTime = currentTime;
    
    sendSerialHeartbeat();   
  #endif
}

/*******************************************************************
 * Main loop funtions
 ******************************************************************/
void loop () {
  
  currentTime = micros();
  deltaTime = currentTime - previousTime;

  measureCriticalSensors();

  // ================================================================
  // 100Hz task loop
  // ================================================================
  if (deltaTime >= 10000) {
    
    frameCounter++;
    
    process100HzTask();

    measureCriticalSensors();
    // ================================================================
    // 50Hz task loop
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
      process50HzTask();
    }
    measureCriticalSensors();

    // ================================================================
    // 10Hz task loop
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
      process10HzTask1();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {
      process10HzTask2();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {
      process10HzTask3();
    }
    measureCriticalSensors();
    
    // ================================================================
    // 1Hz task loop
    // ================================================================
    if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
      process1HzTask();
    }
    measureCriticalSensors();
    
    previousTime = currentTime;
  }
  
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}



