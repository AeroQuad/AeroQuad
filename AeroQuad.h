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

#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

// Flight Software Version
#define VERSION 2.3

#define DEG_2_RAD(d) (d * 0.01745329252)
#define RAD_2_DEG(r) (r * 57.2957795131)

#define G_2_MPS2(g) (g * 9.80665)
#define MPS2_2_G(m) (m * 0.10197162)

//#define BAUD 115200
#define BAUD 111111
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

// Smoothing filter parameters
#define GYRO 0
#define ACCEL 1
#if defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define FINDZERO 9
#else
  #define FINDZERO 49
#endif
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
int minAcro; // Read in from EEPROM, defines min throttle during flips

// Auto level setup
int levelAdjust[2] = {0,0};
int levelLimit; // Read in from EEPROM
int levelOff; // Read in from EEPROM
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
float absoluteHeading = 0;;
float setHeading = 0;

// Altitude Hold
#define TEMPERATURE 0
#define PRESSURE 1
int throttleAdjust = 0;
int throttle = 1000;
int autoDescent = 0;
//#ifndef AeroQuad_v18
int minThrottleAdjust = -50;
int maxThrottleAdjust = 50;
float holdAltitude = 0.0;
int holdThrottle = 1000;
float zDampening = 0.0;
byte storeAltitude = OFF;
byte altitudeHold = OFF;
//#endif

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
#define LASTCHANNEL 6
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

/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/
#define RECEIVERLOOPTIME 100000  // 100ms, 10Hz
#define COMPASSLOOPTIME 103000   // 103ms, ~10Hz
#define ALTITUDELOOPTIME 50000   // 50ms x 2, 10Hz (alternates between temperature and pressure measurements)
#define BATTERYLOOPTIME 100000   // 100ms, 10Hz
#define CAMERALOOPTIME 20000     // 20ms, 50Hz
#define FASTTELEMETRYTIME 10000  // 10ms, 100Hz
#define TELEMETRYLOOPTIME 100000 // 100ms, 10Hz for slower computers/cables (more rough Configurator values)

float G_Dt = 0.002;
// Offset starting times so that events don't happen at the same time
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
unsigned long receiverTime = 0;
unsigned long compassTime = 5000;
unsigned long altitudeTime = 10000;
unsigned long batteryTime = 15000;
unsigned long autoZeroGyroTime = 0;
unsigned long cameraTime = 10000;
unsigned long fastTelemetryTime = 0;
unsigned long telemetryTime = 50000; // make telemetry output 50ms offset from receiver check

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
byte cameraLoop = ON; // Note: stabilization camera software is still under development, moved to Arduino Mega
byte fastTransfer = OFF; // Used for troubleshooting
byte testSignal = LOW;

// **************************************************************
// *************************** EEPROM ***************************
// **************************************************************
// EEPROM storage addresses
#define ROLL_PID_GAIN_ADR 0
#define LEVELROLL_PID_GAIN_ADR 12
#define YAW_PID_GAIN_ADR 24
#define WINDUPGUARD_ADR 36
#define LEVELLIMIT_ADR 40
#define LEVELOFF_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define LEVELPITCHCAL_ADR 60
#define LEVELROLLCAL_ADR 64
#define LEVELZCAL_ADR 68
#define FILTERTERM_ADR 72
#define NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH 76  // needs 8 channel with 3 entries of float (4 byte) -> 96 byte
#define PITCH_PID_GAIN_ADR 172
#define LEVELPITCH_PID_GAIN_ADR 184
#define HEADINGSMOOTH_ADR 200
#define HEADING_PID_GAIN_ADR 204
#define AREF_ADR 216
#define FLIGHTMODE_ADR 220
#define LEVEL_GYRO_ROLL_PID_GAIN_ADR 224
#define LEVEL_GYRO_PITCH_PID_GAIN_ADR 236
#define HEADINGHOLD_ADR 248
#define MINACRO_ADR 252
#define ACCEL1G_ADR 256
#define ALTITUDE_PGAIN_ADR 260
#define ALTITUDE_IGAIN_ADR 264
#define ALTITUDE_DGAIN_ADR 268
#define ALTITUDE_MAX_THROTTLE_ADR 272
#define ALTITUDE_MIN_THROTTLE_ADR 276
#define ALTITUDE_SMOOTH_ADR 280
#define ZDAMP_PGAIN_ADR 284
#define ZDAMP_IGAIN_ADR 288
#define ZDAMP_DGAIN_ADR 292
#define ALTITUDE_WINDUP_ADR 296
#define MAGXMAX_ADR 300
#define MAGXMIN_ADR 304
#define MAGYMAX_ADR 308
#define MAGYMIN_ADR 312
#define MAGZMAX_ADR 316
#define MAGZMIN_ADR 320
#define MCAMERAPITCH_ADR 324
#define MCAMERAROLL_ADR 328
#define MCAMERAYAW_ADR 332
#define CENTERPITCH_ADR 336
#define CENTERROLL_ADR 340
#define CENTERYAW_ADR 344
#define SERVOMINPITCH_ADR 348
#define SERVOMINROLL_ADR 352
#define SERVOMINYAW_ADR 356
#define SERVOMAXPITCH_ADR 360
#define SERVOMAXROLL_ADR 364
#define SERVOMAXYAW_ADR 368
#define GYRO_ROLL_ZERO_ADR 372
#define GYRO_PITCH_ZERO_ADR 376
#define GYRO_YAW_ZERO_ADR 380

float readFloat(int address);                                                                                    // defined in DataStorage.h
void  writeFloat(float value, int address);                                                                      // defined in DataStorage.h
void  readEEPROM(void);                                                                                          // defined in DataStorage.h

void  readPilotCommands(void);                                                                                   // defined in FlightCommand.pde

void  flightControl(void);                                                                                       // defined in FlightControl.pde

int   readApmADC(unsigned char ch_num);                                                                          // defined in ApmAdc.h

float smooth(float currentData, float previousData, float smoothFactor);                                         // defined in Filter.h

void  updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue);                                    // defined in I2C.h
void  sendByteI2C(int deviceAddress, byte dataValue);                                                            // defined in I2C.h
byte  readByteI2C(int deviceAddress);                                                                            // defined in I2C.h
int   readWordI2C(int deviceAddress);                                                                            // defined in I2C.h
int   readWordWaitI2C(int deviceAddress);                                                                        // defined in I2C.h
int   readReverseWordI2C(int deviceAddress);                                                                     // defined in I2C.h
byte  readWhoI2C(int deviceAddress);                                                                             // defined in I2C.h

void  matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[]);  // defined in Matrix.pde
void  matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);                          // defined in Matrix.pde
void  matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);                     // defined in Matrix.pde
void  matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[]);                           // defined in Matrix.pde
void  matrixTranspose3x3(float matrixC[9], float matrixA[9]);                                                    // defined in Matrix.pde
void  matrixInverse3x3(float matrixC[9], float matrixA[9]);                                                      // defined in Matrix.pde

void  readWii(int readWithZeroBias);                                                                             // defined in ReadWii.pde

void  readSensors(void);                                                                                         // defined in Sensors.pde
float arctan2(float y, float x);                                                                                 // defined in Sensors.pde

void  readSerialCommand(void);                                                                                   // defined in SerialCom.pde
void  sendSerialTelemetry(void);                                                                                 // defined in SerialCom.pde
void  printInt(int data);                                                                                        // defined in SerialCom.pde
float readFloatSerial(void);                                                                                     // defined in SerialCom.pde
void  comma(void);                                                                                               // defined in SerialCom.pde

float vectorDotProduct(int length, float vector1[], float vector2[]);                                            // defined in Vector.pde
void  vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3]);                                  // defined in Vector.pde
void  vectorScale(int length, float scaledVector[], float inputVector[], float scalar);                          // defined in Vector.pde
void  vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[]);                                  // defined in Vector.pde
void  vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[]);                             // defined in Vector.pde

void processFlightControlXMode(void); // defined in FlightControl.pde
void processFlightControlPlusMode(void); // defined in FlightControl.pde
void processArdupirateSuperStableMode(void);  // defined in FlightControl.pde
void processAeroQuadStableMode(void);  // defined in FlightControl.pde

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

