/*
  AeroQuad v3.0 - April 2011
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
   If you need additional assitance go to http://www.aeroquad.com/forum.php
   or talk to us live on IRC #aeroquad
*****************************************************************************/

/****************************************************************************
 ************************* Hardware Configuration ***************************
 ****************************************************************************/
// Select which hardware you wish to use with the AeroQuad Flight Software


// 328p platform
//#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
//#define AeroQuad_v1_IDG     // Arduino 2009 with AeroQuad Shield v1.7 and below using IDG yaw gyro
//#define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8 or greater
//#define AeroQuad_Mini       // Arduino Pro Mini with AeroQuad Mini Shield v1.0
//#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors and AeroQuad Shield v1.x
//#define AeroQuad_Paris_v3   // Define along with either AeroQuad_Wii to include specific changes for MultiWiiCopter Paris v3.0 board

// Mega platform
//#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
//#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.0
#define AeroQuadMega_v21    // Arduino Mega with AeroQuad Shield v2.1
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with Oilpan Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider


/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
#define quadXConfig
//#define quadPlusConfig
//#define hexPlusConfig
//#define hexXConfig      // EXPERIMENTAL: not completely re-tested
//#define triConfig
//#define quadY4Config
//#define hexY6Config
//#define octoX8Config
//#define octoPlusConfig  // EXPERIMENTAL: not completely re-tested
//#define octoXConfig     // EXPERIMENTAL: not completely re-tested

//#define CHANGE_YAW_DIRECTION // if you want to reverse the yaw correction direction

// ******************************************************************************************************************************
// Define minimum speed for your motors to run.  this also defines minimum throttle during flips
// Some motors or ESC setups may need more or less define here to start all the montors evenly.
// ******************************************************************************************************************************
#define MIN_ARMED_THROTTLE 1180

//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// In the 3.0 code the motor numbering has changed to simplify motor configuration.
// Please refer to the .h files in ..\Libraries\AQ_FlightControlProcessor to see the new numbering for your flight model
// Also check the http://aeroquad.com/showwiki.php?title=Flight+Configurations for more detail on the 3.0 motor changes 
// the OLD_MOTOR_NUMBERING is compatible  with the 2.x versions of the AeroQuad code and will not need re-ordering to work
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define OLD_MOTOR_NUMBERING // Uncomment this for old motor numbering setup, FOR QUAD +/X MODE ONLY

//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// You must define *only* one of the following flightAngle calculations
// If you only want DCM, then don't define either of the below
// Use FlightAngleARG if you do not have a magnetometer, use DCM if you have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define FlightAngleMARG // EXPERIMENTAL!  Fly at your own risk! Use this if you have a magnetometer installed and enabled HeadingMagHold above
//#define FlightAngleARG // Use this if you do not have a magnetometer installed

//
// *******************************************************************************************************************************
// Optional Sensors
// Warning:  If you enable HeadingHold or AltitudeHold and do not have the correct sensors connected, the flight software may hang
// *******************************************************************************************************************************
#define HeadingMagHold // Enables Magnetometer, gets automatically selected if CHR6DM is defined
#define AltitudeHoldBaro // Enables BMP085 Barometer (experimental, use at your own risk)
//#define AltitudeHoldRangeFinder // EXPERIMENTAL : Enable altitude hold with range finder
//#define RateModeOnly // Use this if you only have a gyro sensor, this will disable any attitude modes.

//
// *******************************************************************************************************************************
// Battery Monitor Options
// For more information on how to setup Battery Monitor please refer to http://aeroquad.com/showwiki.php?title=BatteryMonitor+h
// *******************************************************************************************************************************
//#define BattMonitor            // Enable Battery monitor
//#define BattMonitorAutoDescent // if you want the craft to auto descent when the battery reach the alarm voltage
//#define BattCellCount 3        // set number of Cells (0 == autodetect 1S-3S)
//#define POWERED_BY_VIN         // Uncomment this if your v2.x is powered directly by the vin/gnd of the arduino

//
// *******************************************************************************************************************************
// Optional Receiver
// *******************************************************************************************************************************
//#define RemotePCReceiver // EXPERIMENTAL Use PC as transmitter via serial communicator with XBEE
//#define ReceiverPPM // Use a ppm receiver
//#define ReceiverHWPPM // Use a ppm receiver with HW timer, needs a HW modification (see Libraries/AQ_Receiver/Receiver_HWPPM.h)
// You need to select one of these channel order definitions for PPM receiver
//#define SKETCH_SERIAL_SUM_PPM SERIAL_SUM_PPM_1 //For Graupner/Spektrum (DEFAULT)
//#define SKETCH_SERIAL_SUM_PPM SERIAL_SUM_PPM_2 //For Robe/Hitec/Futaba
//#define SKETCH_SERIAL_SUM_PPM SERIAL_SUM_PPM_3 //For some Hitec/Sanwa/Others

//
// *******************************************************************************************************************************
// Optional telemetry (for debug or ground station tracking purposes)
// For more information on how to setup Telemetry please refer to http://aeroquad.com/showwiki.php?title=Xbee+Installation
// *******************************************************************************************************************************
//#define WirelessTelemetry  // Enables Wireless telemetry on Serial3  // Wireless telemetry enable
//#define BinaryWrite // Enables fast binary transfer of flight data to Configurator
//#define BinaryWritePID // Enables fast binary transfer of attitude PID data
//#define OpenlogBinaryWrite // Enables fast binary transfer to serial1 and openlog hardware

//
// *******************************************************************************************************************************
// Define how many channels are connected from your R/C receiver
// Please note that the flight software currently only supports 6 channels, additional channels will be supported in the future
// Additionally 8 receiver channels are only available when not using the Arduino Uno
// *******************************************************************************************************************************
#define LASTCHANNEL 6
//#define LASTCHANNEL 8 // - warning, this needs to be debugged, incorrect COM behaviour appears when selecting this

//
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

//
// *******************************************************************************************************************************
// On screen display implementation using MAX7456 chip. See MAX7456.h in libraries for more info and configuration.
// For more information on how to setup OSD please refer to http://aeroquad.com/showwiki.php?title=On-Screen-Display
// *******************************************************************************************************************************
//#define OSD
//Choose your (default in case autodetect enabled) video standard: default=NTSC
//#define PAL
//#define AUTODETECT_VIDEO_STANDARD // detect automatically, signal must be present at Arduino powerup!
//#define ShowReticle            // Displays a reticle in the centre of the screen.
//#define ShowFlightTimer        // Displays how long the motors have been armed for since the Arduino was last reset
//#define ShowAttitudeIndicator  // Display the attitude indicator calculated by the AHRS
//#define ShowCallSign           // Show AQ string 
//#define ShowRSSI               // Show Receiver RSSI
//#define feet                   //Comment this line out for altitude measured in metres, uncomment it for feet

// Menu system, currently only usable with OSD
//#define OSD_SYSTEM_MENU



/****************************************************************************
 ****************************************************************************
 ****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************
 ****************************************************************************
 ****************************************************************************/

// Checks to make sure we have the right combinations defined
#if defined(FlightAngleMARG) && !defined(HeadingMagHold)
  #undef FlightAngleMARG
#elif defined(HeadingMagHold) && defined(FlightAngleMARG) && defined(FlightAngleARG)
  #undef FlightAngleARG
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
#ifdef AeroQuad_v1
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // unsuported in v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef HeadingMagHold
  #undef BattMonitor
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuad_v1 specific intialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuad_v1_IDG
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // unsuported in v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef HeadingMagHold
  #undef BattMonitor
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuad_v1_IDG specific intialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuad_v18
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaraion
  #include <Accelerometer_BMA180.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif

  // Battery Monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), 0.9)};
  #endif

  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuad_v18 specific intialization need here
   */
  void initPlatform() {

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

    Wire.begin();
    TWBR = 12;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureAccelSum();
    measureGyroSum();
  }

#endif

#ifdef AeroQuad_Mini
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  
  // Battery Monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), 0.53)};
  #endif

  // unsuported in mini
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuad_Mini specific intialization need here
   */
  void initPlatform() {
    gyroAddress = ITG3200_ADDRESS-1;

    pinMode(LED_Red, OUTPUT);
    digitalWrite(LED_Red, LOW);
    pinMode(LED_Yellow, OUTPUT);
    digitalWrite(LED_Yellow, LOW);

    Wire.begin();
    TWBR = 12;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureAccelSum();
    measureGyroSum();
  }
#endif

#ifdef AeroQuadMega_v1
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  // Special thanks to Wilafau for fixes for this setup
  // http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=11466&viewfull=1#post11466
  // Gyroscope declaration
  #include <Gyroscope_IDG_IDZ500.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL500.h>

  // Reveiver declaration
  #define OLD_RECEIVER_PIN_ORDER
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM

  // unsuported on mega v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef HeadingMagHold
  #undef BattMonitor
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuadMega_v1 specific intialization need here
   */
  void initPlatform() {
    setGyroAref(aref);
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      measureGyro();
      measureAccel();
    }
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
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
//    #define SPARKFUN_5883L_BOB
//    #define HMC5883L
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
      struct BatteryData batteryData[] = {
        BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), 0.0)};// v2 shield powered via VIN (no diode)
    #else
      struct BatteryData batteryData[] = {
        BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5),0.82)}; // v2 shield powered via power jack
    #endif
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  

  /**
   * Put AeroQuadMega_v2 specific intialization need here
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
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureAccelSum();
    measureGyroSum();
    
  }
#endif

#ifdef AeroQuadMega_v21
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  #define SPARKFUN_9DOF
  
  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200_9DOF.h>

  // Accelerometer declaration
  #include <Accelerometer_ADXL345_9DOF.h>

  // Receiver Declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM_Timer

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5883L
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
      struct BatteryData batteryData[] = {
        BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), 0.0)};// v2 shield powered via VIN (no diode)
    #else
      struct BatteryData batteryData[] = {
        BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5),0.82)}; // v2 shield powered via power jack
    #endif
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif  

  /**
   * Put AeroQuadMega_v21 specific intialization need here
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
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureAccelSum();
    measureGyroSum();
  }
#endif

#ifdef ArduCopter
  #define LED_Green 37
  #define LED_Red 35
  #define LED_Yellow 36

  #if defined (HeadingMagHold) || defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    #include <APM_ADC.h>
  #else
    #include <APM_ADC_Optimized.h>
  #endif
  #include <APM_RC.h>
  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_APM.h>

  // Accelerometer Declaration
  #include <Accelerometer_APM.h>

  // Receiver Declaration
  #define RECEIVER_APM

  // Motor Declaration
  #define MOTOR_APM

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif


  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((3.27 / 1024.0) * (10.050 + 3.26) / 3.26), 0.306)};
  #endif

  #undef CameraControl
  #undef OSD

  
  /**
   * Put ArduCopter specific intialization need here
   */
  void initPlatform() {
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);

    initializeADC();
    initRC();

    Wire.begin();
    TWBR = 12;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuad_Wii
  #define LED_Green 13
  #define LED_Red 12
  #define LED_Yellow 12

  #include <Device_I2C.h>

  // Platform Wii declaration
  #include <Platform_Wii.h>

  // Gyroscope declaration
  #include <Gyroscope_Wii.h>

  // Accelerometer declaration
  #include <Accelerometer_WII.h>

  // Receiver declaration
  #define RECEIVER_328P

  // Motor declaration
  #define MOTOR_PWM

  // heading mag hold declaration
  // unsuported on mega v1
  #undef AltitudeHoldBaro
  #undef AltitudeHoldRangeFinder  
  #undef HeadingMagHold
  #undef BattMonitor
  #undef CameraControl
  #undef OSD

  /**
   * Put AeroQuad_Wii specific intialization need here
   */
  void initPlatform() {
     Wire.begin();

     #if defined(AeroQuad_Paris_v3)
       initializeWiiSensors(true);
     #else
       initializeWiiSensors();
     #endif
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      readWiiSensors();
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuadMega_Wii
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  #include <Device_I2C.h>

  // Platform Wii declaration
  #include <Platform_Wii.h>

  // Gyroscope declaration
  #include <Gyroscope_Wii.h>

  // Accelerometer declaration
  #include <Accelerometer_WII.h>

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM

  // heading mag hold declaration
  #ifdef HeadingMagHold
    #define HMC5843
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((5.0 / 1024.0) * (15.0 + 7.5) / 7.5), 0.9)};
  #endif

  #ifdef OSD
    #define MAX7456_OSD
  #endif

  /**
   * Put AeroQuadMega_Wii specific intialization need here
   */
  void initPlatform() {
    Wire.begin();

    initializeWiiSensors();
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      readWiiSensors();
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef AeroQuadMega_CHR6DM
  #define LED_Green 13
  #define LED_Red 4
  #define LED_Yellow 31

  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;

  // Gyroscope declaration
  #include <Gyroscope_CHR6DM.h>

  // Accelerometer declaration
  #include <Accelerometer_CHR6DM.h>

  // Receiver declaration
  #define RECEIVER_MEGA

  // Motor declaration
  #define MOTOR_PWM

  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"

  // Compas declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>

  #ifdef OSD
    #define MAX7456_OSD
  #endif

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif

  // Battery monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((3.27 / 1024.0) * (10.050 + 3.260) / 3.260), 0.9)};
  #endif

  /**
   * Put AeroQuadMega_CHR6DM specific intialization need here
   */
  void initPlatform() {
    Serial1.begin(BAUD);
    PORTD = B00000100;

    Wire.begin();

    chr6dm.resetToFactory();
    chr6dm.setListenMode();
    chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
    chr6dm.requestPacket();

    gyroChr6dm = &chr6dm;
    accelChr6dm = &chr6dm;
    kinematicsChr6dm = &chr6dm;
    compassChr6dm = &chr6dm;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      chr6dm.read();
      measureGyro();
      measureAccel();
    }
  }
#endif

#ifdef APM_OP_CHR6DM
  #define LED_Green 37
  #define LED_Red 35
  #define LED_Yellow 36

  #include <Device_I2C.h>
  #include <Platform_CHR6DM.h>
  CHR6DM chr6dm;

  // Gyroscope declaration
  #include <Gyroscope_CHR6DM.h>

  // Accelerometer declaration
  #include <Accelerometer_CHR6DM.h>

  // Receiver declaration
  #define RECEIVER_APM

  // Motor declaration
  #define MOTOR_APM

  // Kinematics declaration
  #include "Kinematics_CHR6DM.h"

  // Compas declaration
  #define HeadingMagHold
  #define COMPASS_CHR6DM
  #include <Magnetometer_CHR6DM.h>

  // Altitude declaration
  #ifdef AltitudeHoldBaro
    #define BMP085
  #endif
  #ifdef AltitudeHoldRangeFinder
    #define XLMAXSONAR 
  #endif
  

  // Battery monitor declaration
  #ifdef BattMonitor
    struct BatteryData batteryData[] = {
      BM_DEFINE_BATTERY_V(BattCellCount, 0, ((3.27 / 1024.0) * (10.050 + 3.260) / 3.260), 0.306)};
  #endif

  #undef CameraControl
  #undef OSD

  /**
   * Put APM_OP_CHR6DM specific intialization need here
   */
  void initPlatform() {
    pinMode(LED_Red, OUTPUT);
    pinMode(LED_Yellow, OUTPUT);
    pinMode(LED_Green, OUTPUT);

    Serial1.begin(BAUD);
    PORTD = B00000100;

    Wire.begin();

    chr6dm.resetToFactory();
    chr6dm.setListenMode();
    chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
    chr6dm.requestPacket();

    gyroChr6dm = &chr6dm;
    accelChr6dm = &chr6dm;
    kinematicsChr6dm = &chr6dm;
//    tempKinematics.setGyroscope(&gyroSpecific);
    compassChr6dm = &chr6dm;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    if (deltaTime >= 10000) {
      chr6dm.read();
      measureGyro();
      measureAccel();
    }
  }
#endif

//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined (AeroQuadMega_CHR6DM) || defined (APM_OP_CHR6DM)
  // CHR6DM have it's own kinematics, so, initialize in it's scope
#elif defined FlightAngleNewARG
  #include "Kinematics_NewMARG.h"
#elif defined FlightAngleARG
  #include "Kinematics_ARG.h"
#elif defined FlightAngleMARG
  #include "Kinematics_MARG.h"
#else
  #include "Kinematics_DCM.h"
#endif

//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined ReceiverHWPPM
  #include <Receiver_HWPPM.h>
#elif defined ReceiverPPM
  #include <Receiver_PPM.h>
#elif defined (AeroQuad_Mini) && (defined (hexPlusConfig) || defined (hexXConfig) || defined (hexY6Config))
  #include <Receiver_PPM.h>
#elif defined RemotePCReceiver
  #include <Receiver_RemotePC.h>
#elif defined RECEIVER_328P
  #include <Receiver_328p.h>
#elif defined RECEIVER_MEGA
  #include <Receiver_MEGA.h>
#elif defined RECEIVER_APM
  #include <Receiver_APM.h>
#endif


//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined triConfig
  #include <Motors_Tri.h>
#elif defined MOTOR_PWM
  #include <Motors_PWM.h>
#elif defined MOTOR_PWM_Timer
  #include <Motors_PWM_Timer.h>
#elif defined MOTOR_APM
  #include <Motors_APM.h>
#elif defined MOTOR_I2C
  #include <Motors_I2C.h>
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined (HMC5843)
  #include <Magnetometer_HMC5843.h>
#elif defined (HMC5883L)  
  #include <Magnetometer_HMC5883L.h>
#elif defined (COMPASS_CHR6DM)
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined (BMP085)
  #include <BarometricSensor_BMP085.h>
#endif
#if defined (XLMAXSONAR)
  #include <MaxSonarRangeFinder.h>
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor
  #include <BatteryMonitor.h>
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#ifdef CameraControl
  #include <CameraStabilizer_Aeroquad.h>
#endif


//********************************************************
//******** FLIGHT CONFIGURATION DECLARATION **************
//********************************************************
#if defined quadXConfig
  #include "FlightControlQuadX.h"
#elif defined quadPlusConfig
  #include "FlightControlQuadPlus.h"
#elif defined hexPlusConfig
  #include "FlightControlHexPlus.h"
#elif defined hexXConfig
  #include "FlightControlHexX.h"
#elif defined triConfig
  #include "FlightControlTri.h"
#elif defined quadY4Config
  #include "FlightControlQuadY4.h"
#elif defined hexY6Config
  #include "FlightControlHexY6.h"
#elif defined octoX8Config
  #include "FlightControlOctoX8.h"
#elif defined octoXConfig
  #include "FlightControlOctoX.h"
#elif defined octoPlusConfig
  #include "FlightControlOctoPlus.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
  #include <Device_SPI.h>
  #include "OSDDisplayController.h"
  #include "MAX7456.h"
  #ifdef OSD_SYSTEM_MENU
    #include "OSDMenu.h"
  #endif
#else  
    #undef OSD_SYSTEM_MENU  // can't use menu system without an osd, 
#endif

//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined (WirelessTelemetry) 
  #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
    #define SERIAL_PORT Serial3
  #else    // force 328p to use the normal port
    #define SERIAL_PORT Serial
  #endif
#else  
  #define SERIAL_PORT Serial
#endif  

// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"
#include "SerialCom.h"



/**
 * Main setup function, called one time at bootup
 * initalize all system and sub system of the 
 * Aeroquad
 */
void setup() {
  SERIAL_BEGIN(BAUD);
  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);

  #ifdef CHANGE_YAW_DIRECTION
    YAW_DIRECTION = -1;
  #endif

  // Read user values from EEPROM
  readEEPROM(); // defined in DataStorage.h
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
    initializeEEPROM();
    writeEEPROM();
  }

  initPlatform();

  // Configure motors
  #if defined(quadXConfig) || defined(quadPlusConfig) || defined(quadY4Config) || defined(triConfig)
     initializeMotors(FOUR_Motors);
  #elif defined(hexPlusConfig) || defined(hexXConfig) || defined (hexY6Config)
     initializeMotors(SIX_Motors);
  #elif defined (octoX8Config) || defined (octoXConfig) || defined (octoPlusConfig)
     initializeMotors(EIGHT_Motors);
  #endif

  // Setup receiver pins for pin change interrupts
  initializeReceiver(LASTCHANNEL);
  initReceiverFromEEPROM();

  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initializeGyro(); // defined in Gyro.h
  initializeAccel(); // defined in Accel.h
  initSensorsZeroFromEEPROM();

  // Calibrate sensors
  calibrateGyro();
  computeAccelBias();
  zeroIntegralError();

  // Flight angle estimation
  #ifdef HeadingMagHold
    initializeMagnetometer();
    initializeKinematics(getHdgXY(XAXIS), getHdgXY(YAXIS));
  #else
    initializeKinematics(1.0, 0.0);  // with no compass, DCM matrix initalizes to a heading of 0 degrees
  #endif
  // Integral Limit for attitude mode
  // This overrides default set in readEEPROM()
  // Set for 1/2 max attitude command (+/-0.75 radians)
  // Rate integral not used for now
  PID[ATTITUDE_XAXIS_PID_IDX].windupGuard = 0.375;
  PID[ATTITUDE_YAXIS_PID_IDX].windupGuard = 0.375;

  // Optional Sensors
  #ifdef AltitudeHoldBaro
    initializeBaro();
  #endif
  #ifdef AltitudeHoldRangeFinder
    inititalizeRangeFinder(ALTITUDE_RANGE_FINDER_INDEX);
  #endif


  // Battery Monitor
  #ifdef BattMonitor
    // batteryMonitorAlarmVoltage updated in readEEPROM()
    initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
    vehicleState |= BATTMONITOR_ENABLED;
  #endif

  // Camera stabilization setup
  #if defined (CameraControl)
    initializeCameraStabilization();
    setmCameraRoll(318.3); // Need to figure out nice way to reverse servos
    setCenterRoll(1500); // Need to figure out nice way to set center position
    setmCameraPitch(318.3);
    setCenterPitch(1300);
    vehicleState |= CAMERASTABLE_ENABLED;
  #endif

  #if defined(MAX7456_OSD)
    initializeSPI();
    initializeOSD();
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

  setupFourthOrder();

  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
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

  measureCriticalSensors();

  // Main scheduler loop set for 100hz
  if (deltaTime >= 10000) {

    frameCounter++;

    // ================================================================
    // 100hz task loop
    // ================================================================
    if (frameCounter %   1 == 0) {  //  100 Hz tasks
  
      G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
      hundredHZpreviousTime = currentTime;
      
      evaluateMetersPerSec();
      evaluateGyroRate();

      float filteredAccelRoll = computeFourthOrder(meterPerSec[XAXIS], &fourthOrder[AX_FILTER]);
      float filteredAccelPitch = computeFourthOrder(meterPerSec[YAXIS], &fourthOrder[AY_FILTER]);
      float filteredAccelYaw = computeFourthOrder(meterPerSec[ZAXIS], &fourthOrder[AZ_FILTER]);
      
      // ****************** Calculate Absolute Angle *****************
      #if defined FlightAngleNewARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccelRoll,
                            filteredAccelPitch,
                            filteredAccelYaw,
                            0.0,
                            0.0,
                            0.0,
                            G_Dt);

      #elif defined HeadingMagHold && defined FlightAngleMARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccelRoll,
                            filteredAccelPitch,
                            filteredAccelYaw,
                            getMagnetometerRawData(XAXIS),
                            getMagnetometerRawData(YAXIS),
                            getMagnetometerRawData(ZAXIS),
                            G_Dt);
      #elif defined FlightAngleARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccelRoll,
                            filteredAccelPitch,
                            filteredAccelYaw,
                            0.0,
                            0.0,
                            0.0,
                            G_Dt);
      #elif defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccelRoll,
                            filteredAccelPitch,
                            filteredAccelYaw,
                            accelOneG,
                            getHdgXY(XAXIS),
                            getHdgXY(YAXIS),
                            G_Dt);
      #elif !defined HeadingMagHold && !defined FlightAngleMARG && !defined FlightAngleARG
        calculateKinematics(gyroRate[XAXIS],
                            gyroRate[YAXIS],
                            gyroRate[ZAXIS],
                            filteredAccelRoll,
                            filteredAccelPitch,
                            filteredAccelYaw,
                            accelOneG,
                            0.0,
                            0.0,
                            G_Dt);
      #endif

      // Combines external pilot commands and measured sensor data to generate motor commands
      processFlightControl();

      #ifdef BinaryWrite
        if (fastTransfer == ON) {
          // write out fastTelemetry to Configurator or openLog
          fastTelemetry();
        }
      #endif
      
      #ifdef AltitudeHoldRangeFinder
        readRangeFinderDistanceSum(ALTITUDE_RANGE_FINDER_INDEX);
      #endif
    }

    // ================================================================
    // 50hz task loop
    // ================================================================
    if (frameCounter %   2 == 0) {  //  50 Hz tasks

      G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
      fiftyHZpreviousTime = currentTime;

      // Reads external pilot commands and performs functions based on stick configuration
      readPilotCommands(); // defined in FlightCommand.pde

      #if defined AltitudeHoldBaro
        measureBaro(); // defined in altitude.h
      #endif
      #ifdef AltitudeHoldRangeFinder
        evaluateDistanceFromSample(ALTITUDE_RANGE_FINDER_INDEX);
      #endif

      #if defined(CameraControl)
        cameraControlSetPitch(kinematicsAngle[YAXIS]);
        cameraControlSetRoll(kinematicsAngle[XAXIS]);
        cameraControlSetYaw(kinematicsAngle[ZAXIS]);
        cameraControlMove();
      #endif
    }

    // ================================================================
    // 10hz task loop
    // ================================================================
    if (frameCounter %  10 == 0) {  //   10 Hz tasks

      G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
      tenHZpreviousTime = currentTime;

      #if defined(HeadingMagHold)
        measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
      #endif
      #if defined(BattMonitor)
        measureBatteryVoltage(G_Dt);
      #endif
      

      // Listen for configuration commands and reports telemetry
      readSerialCommand(); // defined in SerialCom.pde
      sendSerialTelemetry(); // defined in SerialCom.pde

      #ifdef OSD_SYSTEM_MENU
        updateOSDMenu();
      #endif

      #ifdef MAX7456_OSD
        updateOSD();
      #endif
    }

    previousTime = currentTime;
  }
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}


