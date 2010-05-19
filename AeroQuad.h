/*
  AeroQuad v1.8 - May 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

#ifndef AEROQUAD_H
#define AEROQUAD_H
#include <stdlib.h>
#include <math.h>
#include "WProgram.h"

#define BAUD 115200
#define LEDPIN 13
#define ON 1
#define OFF 0

// Basic axis definitions
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define ZAXIS 2
#define LASTAXIS 3
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
#define HEADING 5

// Smoothing filter parameters
#define GYRO 0
#define ACCEL 1
float smoothTransmitter[6];
float smoothFactor[2];
float smoothHeading;

// Auto level setup
int levelAdjust[2] = {0,0};
int levelLimit; // Read in from EEPROM
int levelOff; // Read in from EEPROM
// Scale to convert 1000-2000 PWM to +/- 45 degrees
float mLevelTransmitter = 0.09;
float bLevelTransmitter = -135;

// Heading hold
// aref / 1024 = voltage per A/D bit
// 0.002 = V / deg/sec (from gyro data sheet)
float headingScaleFactor;
float heading = 0; // measured heading from yaw gyro (process variable)
float headingHold = 0; // calculated adjustment for quad to go to heading (PID output)
float currentHeading = 0; // current heading the quad is set to (set point)

// Altitude Adjust
float zAccelHover;
int throttleAdjust;
int throttleAdjustGain = 10; // Look to make this a command setting
int minThrottleAdjust = -200;
int maxThrottleAdjust = 200;

// Camera stabilization variables
// Note: stabilization camera software is still under development
#ifdef Camera
  #define ROLLCAMERAPIN 12
  #define PITCHCAMERAPIN 13
  // map +/-90 degrees to 1000-2000
  float mCamera = 5.556;
  float bCamera = 1500;
  Servo rollCamera;
  Servo pitchCamera;
#endif

// ESC Calibration
byte calibrateESC = 0;
int testCommand = 1000;

// Communication
char queryType = 'X';
byte tlmType = 0;
char string[32];
byte armed = 0;
byte safetyCheck = 0;
byte update = 0;

/**************************************************************/
/******************* Loop timing parameters *******************/
/**************************************************************/
#define RECEIVERLOOPTIME 100
#define TELEMETRYLOOPTIME 100
#define FASTTELEMETRYTIME 10
#define AILOOPTIME 2
#define CONTROLLOOPTIME 2

float AIdT = AILOOPTIME / 1000.0;
float controldT = CONTROLLOOPTIME / 1000.0;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
unsigned long receiverTime = 0;
unsigned long telemetryTime = 50; // make telemetry output 50ms offset from receiver check
unsigned long analogInputTime = 0;
unsigned long controlLoopTime = 1; // offset control loop from analog input loop by 1ms
unsigned long cameraTime = 0;
unsigned long fastTelemetryTime = 0;
unsigned long autoZeroGyroTime = 0;

/**************************************************************/
/********************** Debug Parameters **********************/
/**************************************************************/
// Enable/disable control loops for debug
//#define DEBUG
byte receiverLoop = ON;
byte telemetryLoop = ON;
byte analogInputLoop = ON;
byte controlLoop = ON;
byte cameraLoop = OFF; // Note: stabilization camera software is still under development, moved to Arduino Mega
byte fastTransfer = ON;
byte testSignal = LOW;
// Measured test signal with an oscilloscope:
// All loops on = 2.4 ms
// Analog Input and Control loop on = 2.0 ms
// Analog Input loop on = 1.6 ms
// Analog Input loop on = 1 ms (no comp filter)
// Analog Input loop on = 0.6 ms (comp filter only)
// Control loop on = 0.4 ms
// Receiver loop = 0.4 ms
// Telemetry loop = .04 ms (with no telemetry transmitted)
// Fast Telemetry Transfer (sending 12 bytes = 1.1 ms, sending 14 bytes = 1.3 ms, sending 16 bytes = 1.45 ms, sending 18 bytes = 1.625 ms) 2 bytes = 0.175 ms

#endif
