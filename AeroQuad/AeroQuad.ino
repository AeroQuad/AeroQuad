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

#error Dev branch is broke for the current development, please use official release v3.2 of flight software and configurator!

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

#if defined (AeroQuadMega_v2) || defined (AeroQuadMega_v21) || defined (AeroQuadSTM32)
  
  #define USE_400HZ_ESC			// For ESC that support 400Hz update rate, ESC OR PLATFORM MAY NOT SUPPORT IT
  
  #define HeadingMagHold		// Enables Magnetometer, gets automatically selected if CHR6DM is defined
  #define AltitudeHoldBaro		// Enables Barometer
  #define AltitudeHoldRangeFinder	// Enables Altitude Hold with range finder, not displayed on the configurator (yet)
  #define AutoLanding			// Enables auto landing on channel AUX3 of the remote, NEEDS AltitudeHoldBaro AND AltitudeHoldRangeFinder to be defined
  
  //
  // *******************************************************************************************************************************
  // GPS Options
  // *******************************************************************************************************************************
  #define UseGPS		        // Enables GPS (for mega v2.0/v2.1 on Serial1 & AeroQuad32 on Serial2)

  // Device specific settings
  //#define UseGPSMTKBINARY   // Set MTK devices to binary protocol (only DiyDrones MTK1.6 protocol supported)

  //
  // *******************************************************************************************************************************
  // Battery Monitor Options
  // For more information on how to setup the Battery Monitor please refer to http://aeroquad.com/showwiki.php?title=Battery+Monitor
  // *******************************************************************************************************************************
  //#define BattMonitor			  // Enables Battery monitor
  //#define BattMonitorAutoDescent  // NEED BattMonitor defined. If you want the craft to auto descent when the battery reaches the alarm voltage
  //#define POWERED_BY_VIN          // NEED BattMonitor defined. Uncomment this if your v2.x shield is powered directly by the Vin/Gnd of the arduino
  //
  // Advanced configuration. Please refer to the wiki for instructions.
  //#define BattCustomConfig DEFINE_BATTERY(0,A4,51.8,0,A3,180.3,0)
  
  
  //#define UseAnalogRSSIReader	// Reads RSSI for receiver failsafe, NEEDS A RECEIVER WITH FAILSAVE CONNECTED ON PIN A6 OF THE SHIELD
  //#define UseEzUHFRSSIReader	// Reads RSSI and Signal quality on channel 7(RSSI) and 8(Signal Quality) of the EzUHF receiver (Receiver have to be configures this way)
  //#define UseSBUSRSSIReader		



  //
  // *******************************************************************************************************************************
  // Optional telemetry (for debug or ground station tracking purposes)
  // For more information on how to setup Telemetry please refer to http://aeroquad.com/showwiki.php?title=Wireless+Connection
  // *******************************************************************************************************************************
  //#define WirelessTelemetry	// Enables Wireless telemetry on Serial3  // Wireless telemetry enable
  
  //#define MavLink               // Enables the MavLink protocol
  //#define MAV_SYSTEM_ID 100		// Needs to be enabled when using MavLink, used to identify each of your copters using MavLink
  								// If you've only got one, leave the default value unchanged, otherwise make sure that each copter has a different ID 
  
  //#define CONFIG_BAUDRATE 19200 // overrides default baudrate for serial port (Configurator/MavLink/WirelessTelemetry)
  
  //  #define SlowTelemetry			// Enables audio channel telemetry on Serial2
  //  #define SoftModem             // Enable usage of DAC as modem on AQ32 instead of Serial 2
  //  #define SOFTMODEM_FSKv2       // Enable non standard FSK frequencies used by FSKv2 module (TCM3105 at 8Mhz)

  #define CameraControl
  #define CameraTXControl  // need to have CameraControl to work

  #define OSD
  #define ShowRSSI                  // This REQUIRES a RSSI reader
  #define PAL                       // uncomment this to default to PAL video
  #define AUTODETECT_VIDEO_STANDARD // detect automatically, signal must be present at Arduino powerup!
  #define CALLSIGN "AQ"             // Show (optional) callsign
  #define ShowAttitudeIndicator     // Display the attitude indicator calculated by the AHRS
  #define ShowLandingIndicator      // Display the landing indicator calculated via barometer
  #define USUnits                   // Enable for US units (feet,miles,mph), leave uncommented for metric units (meter,kilometer,km/h)
//  #define OSD_LOADFONT              // Include MAX7456 font into binary, give & on serial to upload

  #define OSD_SYSTEM_MENU           // Menu system, currently only usable with OSD or SERIAL_LCD

//  #define SERIAL_LCD Serial3  

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
//  #define CHANGE_YAW_DIRECTION
  #include <Motors_328p.h>
  
  #include <FlightConfig328p.h>
  
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

  // Motor declaration
  #define CHANGE_YAW_DIRECTION
  #include <Motors_328p.h>
  
  #include <FlightConfig328p.h>
  
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

#ifdef AeroQuadMega_v2
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_BMA180.h>

  // Receiver Declaration
  // Receiver declaration
  #include <Receiver_MEGA.h>

  // Motor declaration
  #include <Motors_MEGA.h>
  
  #include <FlightConfigMEGA.h>
  
  // heading mag hold declaration
//  #define HeadingMagHold
  #ifdef HeadingMagHold
    #include <Compass.h>
//    #define SPARKFUN_5883L_BOB
    #define HMC5843
  #endif

  // Altitude declaration
//  #define AltitudeHoldBaro
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
#include "Kinematics_ARG.h"

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
//#if defined(ReceiverHWPPM)
//  #include <Receiver_HWPPM.h>
//#elif defined(ReceiverPPM)
//  #include <Receiver_PPM.h>
//#elif defined(AeroQuad_Mini) && (defined(HEX_PLUS) || defined(HEX_X) || defined(HEX_Y6))
//  #include <Receiver_PPM.h>
//#elif defined(RemotePCReceiver)
//  #include <Receiver_RemotePC.h>
//#elif defined(ReceiverSBUS)
//  #include <Receiver_SBUS.h>
//#elif defined(RECEIVER_328P)
//  #include <Receiver_328p.h>
//#elif defined(RECEIVER_MEGA)
//  #include <Receiver_MEGA.h>
//#elif defined(RECEIVER_APM)
//  #include <Receiver_APM.h>
//#elif defined(RECEIVER_STM32PPM)
//  #include <Receiver_STM32PPM.h>  
//#elif defined(RECEIVER_STM32)
//  #include <Receiver_STM32.h>  
//#endif

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
//#if defined(TRI)
//  #if defined (MOTOR_STM32)
//    #define MOTORS_STM32_TRI
//    #include <Motors_STM32.h>    
//  #else
//    #include <Motors_Tri.h>
//  #endif
//#elif defined(MOTOR_PWM)
//  #include <Motors_PWM.h>
//#elif defined(MOTOR_PWM_Timer)
//  #include <Motors_PWM_Timer.h>
//#elif defined(MOTOR_APM)
//  #include <Motors_APM.h>
//#elif defined(MOTOR_I2C)
//  #include <Motors_I2C.h>
//#elif defined(MOTOR_STM32)
//  #include <Motors_STM32.h>    
//#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843)
  #include <HeadingFusionProcessorMARG.h>
  #include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L)
  #include <HeadingFusionProcessorMARG.h>
  #include <Magnetometer_HMC5883L.h>
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
  #include <BarometricSensor_BMP085.h>
#elif defined(MS5611)
 #include <BarometricSensor_MS5611.h>
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
#if defined(WirelessTelemetry) 
  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define SERIAL_PORT Serial3
  #else    // force 328p to use the normal port
    #define SERIAL_PORT Serial
  #endif
#else  
  #if defined(SERIAL_USES_USB)   // STM32 Maple
    #define SERIAL_PORT SerialUSB
    #undef BAUD
    #define BAUD
  #else
    #define SERIAL_PORT Serial
  #endif
#endif  

#ifdef SlowTelemetry
  #include <AQ_RSCode.h>
#endif

#ifdef SoftModem
  #include <AQ_SoftModem.h>
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
  
  readEEPROM(); // defined in DataStorage.h
  boolean firstTimeBoot = false;
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
    initializeEEPROM();
    writeEEPROM();
    storeSensorsZeroToEEPROM();
    firstTimeBoot = true;
  }
  
  initPlatform();

  initializeMotors(LASTMOTOR);

//  initializeReceiver();
  (*initializeReceiver[receiverTypeUsed])();

  initReceiverFromEEPROM();
  
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
  
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;
  
  // Flight angle estimation
  initializeKinematics();

  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer();
    initializeHeadingFusion();
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
    PID[SONAR_ALTITUDE_HOLD_PID_IDX].windupGuard = PID[BARO_ALTITUDE_HOLD_PID_IDX].windupGuard;
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

  #if defined(BinaryWrite) || defined(BinaryWritePID)
    #ifdef OpenlogBinaryWrite
      binaryPort = &Serial1;
      binaryPort->begin(115200);
      delay(1000);
    #else
     binaryPort = &Serial;
    #endif
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
    
  calculateKinematics(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], G_Dt);
  
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    zVelocity = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS] - runtimeZBias;
    if (!runtimaZBiasInitialized) {
      runtimeZBias = (filteredAccel[ZAXIS] * (1 - accelOneG * invSqrt(isq(filteredAccel[XAXIS]) + isq(filteredAccel[YAXIS]) + isq(filteredAccel[ZAXIS])))) - runTimeAccelBias[ZAXIS];
      runtimaZBiasInitialized = true;
    }
    estimatedZVelocity += zVelocity;
    estimatedZVelocity = (velocityCompFilter1 * zVelocity) + (velocityCompFilter2 * estimatedZVelocity);
  #endif    

  #if defined(AltitudeHoldBaro)
    measureBaroSum(); 
    if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  //  50 Hz tasks
      evaluateBaroAltitude();
    }
  #endif
        
  processFlightControl();
  
  
  #if defined(BinaryWrite)
    if (fastTransfer == ON) {
      // write out fastTelemetry to Configurator or openLog
      fastTelemetry();
    }
  #endif      
  
  #ifdef SlowTelemetry
    updateSlowTelemetry100Hz();
  #endif

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
     
    measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
    
    calculateHeading();
    
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

    // ================================================================
    // 50Hz task loop
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
      process50HzTask();
    }

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
    
    // ================================================================
    // 1Hz task loop
    // ================================================================
    if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
      process1HzTask();
    }
    
    previousTime = currentTime;
  }
  
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}

