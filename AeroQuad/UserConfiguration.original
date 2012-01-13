/*
  AeroQuad v3.0 - December 2011
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
#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.0
//#define AeroQuadMega_v21    // Arduino Mega with AeroQuad Shield v2.1
//#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors and AeroQuad Shield v2.x
//#define ArduCopter          // ArduPilot Mega (APM) with Oilpan Sensor Board
//#define AeroQuadMega_CHR6DM // Clean Arduino Mega with CHR6DM as IMU/heading ref.
//#define APM_OP_CHR6DM       // ArduPilot Mega with CHR6DM as IMU/heading ref., Oilpan for barometer (just uncomment AltitudeHold for baro), and voltage divider


/****************************************************************************
 *********************** Define Flight Configuration ************************
 ****************************************************************************/
// Use only one of the following definitions
//#define quadXConfig
//#define quadPlusConfig
//#define hexPlusConfig
//#define hexXConfig      // EXPERIMENTAL: not completely re-tested
//#define triConfig
//#define quadY4Config
#define hexY6Config
//#define octoX8Config
//#define octoPlusConfig  // EXPERIMENTAL: not completely re-tested
//#define octoXConfig     // EXPERIMENTAL: not completely re-tested

//#define CHANGE_YAW_DIRECTION // if you want to reverse the yaw correction direction

//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// In the 3.0 code the motor numbering has changed to simplify motor configuration.
// Please refer to the .h files in ..\Libraries\AQ_FlightControlProcessor to see the new numbering for your flight model
// Also check the http://www.aeroquad.com/showwiki.php "Build Instructions" for more detail on the 3.0 motor changes 
// the OLD_MOTOR_NUMBERING is compatible  with the 2.x versions of the AeroQuad code and will not need re-ordering to work
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define OLD_MOTOR_NUMBERING // Uncomment this for old motor numbering setup, FOR QUAD +/X MODE ONLY

//
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// You must define *only* one of the following flightAngle calculations
// If you only want DCM, then don't define either of the below
// Use FlightAngleARG if you do not have a magnetometer, use DCM if you have a magnetometer installed
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//#define FlightAngleMARG // EXPERIMENTAL!  Fly at your own risk! Use this if you have a magnetometer installed and enabled HeadingMagHold above//
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
#define BattMonitor            // Enable Battery monitor
#define BattMonitorAutoDescent // if you want the craft to auto descent when the battery reach the alarm voltage
#define BattCellCount 3        // set number of Cells (0 == autodetect 1S-3S)
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
//#define PAL                       // uncomment this to default to PAL video
//#define AUTODETECT_VIDEO_STANDARD // detect automatically, signal must be present at Arduino powerup!
//#define CALLSIGN "AeroQD"         // Show (optional) callsign
//#define ShowAttitudeIndicator     // Display the attitude indicator calculated by the AHRS
//#define feet                      // Leave commented for altitude measured in metres, uncomment for feet

//#define OSD_SYSTEM_MENU           // Menu system, currently only usable with OSD

/****************************************************************************
 ****************************************************************************
 ****************************************************************************
 ********************* End of User Definition Section ***********************
 ****************************************************************************
 ****************************************************************************
 ****************************************************************************/
