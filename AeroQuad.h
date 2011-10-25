/*
  AeroQuad v2.5 Beta 1 - July 2011
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

#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

// Flight Software Version
#define VERSION 2.5

#define BAUD 115200
//#define BAUD 111111 // use this to be compatible with USB and XBee connections
//#define BAUD 57600
#define LEDPIN 13
#define ON 1
#define OFF 0

#if defined(APM_OP_CHR6DM) || defined(ArduCopter) 
  #define LED_Red 35
  #define LED_Yellow 36
  #define LED_Green 37
  #define RELE_pin 47
  #define SW1_pin 41
  #define SW2_pin 40
  #define BUZZER 9
  #define PIANO_SW1 42
  #define PIANO_SW2 43
#endif
#ifdef AeroQuadMega_v2  
  #define LED2PIN 4
  #define LED3PIN 31
#else
  #define LED2PIN 12
  #define LED3PIN 12
#endif

// Basic axis definitions
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define AUX2 6
#define AUX3 7
#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2
#define LASTAXIS 3
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
#define HEADING 5
#define LEVELGYROROLL 6
#define LEVELGYROPITCH 7
#define ALTITUDE 8
#define ZDAMPENING 9

// PID Variables
struct PIDdata {
  float P, I, D;
  float lastPosition;
  // AKA experiments with PID
  float previousPIDTime;
  bool firstPass;
  bool typePID;
  float integratedError;
  float windupGuard; // Thinking about having individual wind up guards for each PID
} PID[10];
// This struct above declares the variable PID[] to hold each of the PID values for various functions
// The following constants are declared in AeroQuad.h
// ROLL = 0, PITCH = 1, YAW = 2 (used for Arcobatic Mode, gyros only)
// ROLLLEVEL = 3, PITCHLEVEL = 4, LEVELGYROROLL = 6, LEVELGYROPITCH = 7 (used for Stable Mode, accels + gyros)
// HEADING = 5 (used for heading hold)
// ALTITUDE = 8 (used for altitude hold)
// ZDAMPENING = 9 (used in altitude hold to dampen vertical accelerations)
float windupGuard; // Read in from EEPROM

// PID types
#define NOTYPE 0
#define TYPEPI 1

// Smoothing filter parameters
#define GYRO 0
#define ACCEL 1
#define FINDZERO 49
float smoothHeading;

// Sensor pin assignments
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5
#define AZPIN 12 // Auto zero pin for IDG500 gyros

// Motor control variables
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define MOTORID1 0		
#define MOTORID2 1		
#define MOTORID3 2		
#define MOTORID4 3		
#define MOTORID5 4		
#define MOTORID6 5
#define MINCOMMAND 1000
#define MAXCOMMAND 2000
#if defined(plusConfig) || defined(XConfig)
  #define LASTMOTOR 4
#endif
#if defined(HEXACOAXIAL) || defined(HEXARADIAL)
  #define LASTMOTOR 6
#endif

// Analog Reference Value
// This value provided from Configurator
// Use a DMM to measure the voltage between AREF and GND
// Enter the measured voltage below to define your value for aref
// If you don't have a DMM use the following:
// AeroQuad Shield v1.7, aref = 3.0
// AeroQuad Shield v1.6 or below, aref = 2.8
float aref; // Read in from EEPROM

// Flight Mode
#define ACRO 0
#define STABLE 1
byte flightMode;
unsigned long frameCounter = 0; // main loop executive frame counter
int minAcro; // Read in from EEPROM, defines min throttle during flips
#define PWM2RAD 0.002 //  Based upon 5RAD for full stick movement, you take this times the RAD to get the PWM conversion factor

// Auto level setup
//float levelAdjust[2] = {0.0,0.0};
//int levelAdjust[2] = {0,0};
//int levelLimit; // Read in from EEPROM
//int levelOff; // Read in from EEPROM
// Scale to convert 1000-2000 PWM to +/- 45 degrees
//float mLevelTransmitter = 0.09;
//float bLevelTransmitter = -135;

#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
  float CHR_RollAngle;
  float CHR_PitchAngle;
#endif

// Heading hold
byte headingHoldConfig;
//float headingScaleFactor;
float commandedYaw = 0;
float headingHold = 0; // calculated adjustment for quad to go to heading (PID output)
float heading = 0; // measured heading from yaw gyro (process variable)
float relativeHeading = 0; // current heading the quad is set to (set point)
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
float absoluteHeading = 0;;
#endif
float setHeading = 0;
unsigned long headingTime = micros();
byte headingHoldState = OFF;

// batteryMonitor & Altutude Hold
int throttle = 1000;
int autoDescent = 0;

// Altitude Hold
#define ALTPANIC 2 // special state that allows immediate turn off of Altitude hold if large throttle changesa are made at the TX
#define ALTBUMP 90 // amount of stick movement to cause an altutude bump (up or down)
#define PANICSTICK_MOVEMENT 250 // 80 if althold on and throttle commanded to move by a gross amount, set PANIC
//#define MINSTICK_MOVEMENT 32 // any movement less than this doesn't not trigger a rest of the holdaltitude
#define TEMPERATURE 0
#define PRESSURE 1
int throttleAdjust = 0;

int minThrottleAdjust = -50;
int maxThrottleAdjust = 50;
float holdAltitude = 0.0;
int holdThrottle = 1000;
float zDampening = 0.0;
byte storeAltitude = OFF;
byte altitudeHold = OFF;

// Receiver variables
#define TIMEOUT 25000
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 100
#define LEVELOFF 100
int delta;

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

// Flight angle variables
float timeConstant;

// ESC Calibration
byte calibrateESC = 0;
int testCommand = 1000;

// Communication
char queryType = 'X';
byte tlmType = 0;
byte armed = OFF;
byte safetyCheck = OFF;
byte update = 0;
HardwareSerial *binaryPort;

/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/
/*
#define RECEIVERLOOPTIME 100000  // 100ms, 10Hz
#define COMPASSLOOPTIME 103000   // 103ms, ~10Hz
#define ALTITUDELOOPTIME 50000   // 50ms x 2, 10Hz (alternates between temperature and pressure measurements)
#define BATTERYLOOPTIME 100000   // 100ms, 10Hz
#define CAMERALOOPTIME 20000     // 20ms, 50Hz
#define FASTTELEMETRYTIME 15000  // 15ms, 67Hz
#define TELEMETRYLOOPTIME 100000 // 100ms, 10Hz for slower computers/cables (more rough Configurator values)
*/

float G_Dt = 0.002;
// Offset starting times so that events don't happen at the same time
// main loop times
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
// sub loop times
//unsigned long oneHZpreviousTime;
unsigned long tenHZpreviousTime;
unsigned long twentyFiveHZpreviousTime;
unsigned long fiftyHZpreviousTime;
unsigned long hundredHZpreviousTime;
// old times.
//unsigned long receiverTime = 0;
//unsigned long compassTime = 5000;
//unsigned long altitudeTime = 10000;
//unsigned long batteryTime = 15000;
//unsigned long autoZeroGyroTime = 0;
#ifdef CameraControl
unsigned long cameraTime = 10000;
#endif
unsigned long fastTelemetryTime = 0;
//unsigned long telemetryTime = 50000; // make telemetry output 50ms offset from receiver check

// jihlein: wireless telemetry defines
/**************************************************************/
/********************** Wireless Telem Port *******************/
/**************************************************************/
#if defined WirelessTelemetry && (defined(AeroQuadMega_v1)     || \
                                  defined(AeroQuadMega_v2)     || \
                                  defined(AeroQuadMega_Wii)    || \
                                  defined(ArduCopter)          || \
                                  defined(AeroQuadMega_CHR6DM) || \
                                  defined(APM_OP_CHR6DM))
  #define SERIAL_BAUD       115200
  #define SERIAL_PRINT      Serial3.print
  #define SERIAL_PRINTLN    Serial3.println
  #define SERIAL_AVAILABLE  Serial3.available
  #define SERIAL_READ       Serial3.read
  #define SERIAL_FLUSH      Serial3.flush
  #define SERIAL_BEGIN      Serial3.begin
#else
  #define SERIAL_BAUD       115200
  #define SERIAL_PRINT      Serial.print
  #define SERIAL_PRINTLN    Serial.println
  #define SERIAL_AVAILABLE  Serial.available
  #define SERIAL_READ       Serial.read
  #define SERIAL_FLUSH      Serial.flush
  #define SERIAL_BEGIN      Serial.begin
#endif

/**************************************************************/
/********************** Debug Parameters **********************/
/**************************************************************/
// Enable/disable control loops for debug
//#define DEBUG
byte receiverLoop = ON;
byte telemetryLoop = ON;
byte sensorLoop = ON;
byte controlLoop = ON;
#ifdef CameraControl
byte cameraLoop = ON; // Note: stabilization camera software is still under development, moved to Arduino Mega
#endif
byte fastTransfer = OFF; // Used for troubleshooting
byte testSignal = LOW;

// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************
// EEPROM storage addresses


typedef struct {
  float p;
  float i;
  float d;
} t_NVR_PID;

typedef struct {
  float slope;
  float offset;
  float smooth_factor;
} t_NVR_Receiver;
typedef struct {    
  t_NVR_PID ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVELROLL_PID_GAIN_ADR;
  t_NVR_PID YAW_PID_GAIN_ADR;
  t_NVR_PID PITCH_PID_GAIN_ADR;
  t_NVR_PID LEVELPITCH_PID_GAIN_ADR;
  t_NVR_PID HEADING_PID_GAIN_ADR;
  t_NVR_PID LEVEL_GYRO_ROLL_PID_GAIN_ADR;
  t_NVR_PID LEVEL_GYRO_PITCH_PID_GAIN_ADR;
  t_NVR_PID ALTITUDE_PID_GAIN_ADR;
  t_NVR_PID ZDAMP_PID_GAIN_ADR;
  t_NVR_Receiver RECEIVER_DATA[LASTCHANNEL];
  
  float WINDUPGUARD_ADR;
  float XMITFACTOR_ADR;
  float GYROSMOOTH_ADR;
  float ACCSMOOTH_ADR;
  float LEVELPITCHCAL_ADR;
  float LEVELROLLCAL_ADR;
  float LEVELZCAL_ADR;
  float FILTERTERM_ADR;
  float HEADINGSMOOTH_ADR;
  float AREF_ADR;
  float FLIGHTMODE_ADR;
  float HEADINGHOLD_ADR;
  float MINACRO_ADR;
  float ACCEL1G_ADR;
//  float ALTITUDE_PGAIN_ADR;
  float ALTITUDE_MAX_THROTTLE_ADR;
  float ALTITUDE_MIN_THROTTLE_ADR;
  float ALTITUDE_SMOOTH_ADR;
//  float ZDAMP_PGAIN_ADR;
  float ALTITUDE_WINDUP_ADR;
  float MAGXMAX_ADR;
  float MAGXMIN_ADR;
  float MAGYMAX_ADR;
  float MAGYMIN_ADR;
  float MAGZMAX_ADR;
  float MAGZMIN_ADR;
  float SERVOMINPITCH_ADR;
  float SERVOMINROLL_ADR;
  float GYRO_ROLL_ZERO_ADR;
  float GYRO_PITCH_ZERO_ADR;
  float GYRO_YAW_ZERO_ADR;
} t_NVR_Data;  

float nvrReadFloat(int address); // defined in DataStorage.h
void nvrWriteFloat(float value, int address); // defined in DataStorage.h
void nvrReadPID(unsigned char IDPid, unsigned int IDEeprom);
void nvrWritePID(unsigned char IDPid, unsigned int IDEeprom);


#define GET_NVR_OFFSET(param) ((int)&(((t_NVR_Data*) 0)->param))
#define readFloat(addr) nvrReadFloat(GET_NVR_OFFSET(addr))
#define writeFloat(value, addr) nvrWriteFloat(value, GET_NVR_OFFSET(addr))
#define readPID(IDPid, addr) nvrReadPID(IDPid, GET_NVR_OFFSET(addr))
#define writePID(IDPid, addr) nvrWritePID(IDPid, GET_NVR_OFFSET(addr))


// external dunction defined
float arctan2(float y, float x); // defined in AQMath.h
void readEEPROM(void); // defined in DataStorage.h
void readPilotCommands(void); // defined in FlightCommand.pde
void readSensors(void); // defined in Sensors.pde
void processFlightControlXMode(void); // defined in FlightControl.pde
void processFlightControlPlusMode(void); // defined in FlightControl.pde
void readSerialCommand(void);  //defined in SerialCom.pde
void sendSerialTelemetry(void); // defined in SerialCom.pde
void printInt(int data); // defined in SerialCom.pde
float readFloatSerial(void); // defined in SerialCom.pde
void sendBinaryFloat(float); // defined in SerialCom.pde
void sendBinaryuslong(unsigned long); // defined in SerialCom.pde
void fastTelemetry(void); // defined in SerialCom.pde
void comma(void); // defined in SerialCom.pde
void processAltitudeHold(void); // defined in FlightControl.pde

#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
float findMode(float *data, int arraySize); // defined in Sensors.pde
#else
int findMode(int *data, int arraySize); // defined in Sensors.pde
#endif

// FUNCTION: return the number of bytes currently free in RAM      
extern int  __bss_end; // used by freemem 
extern int  *__brkval; // used by freemem
int freemem(){
    int free_memory;
    if((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);
    return free_memory;
}

