/*
  AeroQuad v1.4 - September 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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
#include "WProgram.h"

#define BAUD 115200
#define LEDPIN 13

// Auto level setup
int levelAdjust[2] = {0,0};
int levelLimit; // Read in from EEPROM
int levelOff; // Read in from EEPROM

// Heading hold
float heading = 0;
float aref = 2.896 * 3.0; // With 4.7k Ohm resistor

// Camera stabilization variables
// Note: stabilization camera software is still under development
#define ROLLCAMERAPIN 12
#define PITCHCAMERAPIN 13
// map +/-90 degrees to 1000-2000
float mCamera = 5.556;
float bCamera = 1500;
#ifdef Camera
  ServoTimer2 rollCamera;
  ServoTimer2 pitchCamera;
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

// Enable/disable control loops for debug
//#define DEBUG
#define ON 1
#define OFF 0
byte receiverLoop = ON;
byte telemetryLoop = ON;
byte analogInputLoop = ON;
byte controlLoop = ON;
byte cameraLoop = OFF; // Note: stabilization camera software is still under development
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

// Timing
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long deltaTime = 0;
unsigned long receiverTime = 0;
unsigned long telemetryTime = 50; // make telemetry output 50ms offset from receiver check
unsigned long analogInputTime = 0;
unsigned long controlLoopTime = 1; // offset control loop from analog input loop by 1ms
unsigned long cameraTime = 0;
float dt = 0;

#endif
