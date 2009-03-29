/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

// To Do:
// Create automated calibration for Z axis

#include <stdlib.h>
#include <math.h>
#include <EEPROM.h>
#include <ServoTimer2.h>

// ******************** Initialize Variables ********************
#define BAUD 115200
#define LEDPIN 13

// Sensor pin assignments
// Original orientation
#define PITCHACCELPIN 0
#define ROLLACCELPIN 1
#define ZACCELPIN 2
#define PITCHRATEPIN 3
#define ROLLRATEPIN 4
#define YAWRATEPIN 5
// New orientation
/*#define PITCHACCELPIN 4
#define ROLLACCELPIN 5
#define ZACCELPIN 3
#define PITCHRATEPIN 0
#define ROLLRATEPIN 1
#define YAWRATEPIN 2*/
#include "WProgram.h"
void setup();
void loop ();
float readFloat(int address);
void writeFloat(float value, int address);
void readEEPROM();
void configureFilter(float timeConstant);
float filterData(float previousAngle, int gyroADC, int accelADC, float *filterTerm, float dt);
int smooth(int currentData, int previousData, float smoothFactor);
void configureMotors();
void commandMotors();
void commandAllMotors(int motorCommand);
void pulseMotors(byte quantity);
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters);
void zeroIntegralError();
int findMode(int *data, int arraySize);
void zeroGyros();
void zeroAccelerometers();
int limitRange(int data, int minLimit, int maxLimit);
void readSerialCommand();
float readFloatSerial();
void sendSerialTelemetry();
void comma();
void configureTransmitter();
void readTransmitter();
int gyroChannel[3] = {ROLLRATEPIN, PITCHRATEPIN, YAWRATEPIN};
int accelChannel[3] = {ROLLACCELPIN, PITCHACCELPIN, ZACCELPIN};

// EEPROM storage
#define PGAIN_ADR 0
#define IGAIN_ADR 4
#define DGAIN_ADR 8
#define LEVEL_PGAIN_ADR 12
#define LEVEL_IGAIN_ADR 16
#define LEVEL_DGAIN_ADR 20
#define YAW_PGAIN_ADR 24
#define YAW_IGAIN_ADR 28
#define YAW_DGAIN_ADR 32
#define WINDUPGUARD_ADR 36
#define LEVELLIMIT_ADR 40
#define LEVELINTERVAL_ADR 44
#define XMITFACTOR_ADR 48
#define GYROSMOOTH_ADR 52
#define ACCSMOOTH_ADR 56
#define LEVELPITCHCAL_ADR 60
#define LEVELROLLCAL_ADR 64
#define LEVELZCAL_ADR 68
#define FILTERTERM_ADR 72
/*#define ROLLCH_ADR 76
#define PITCHCH_ADR 80
#define YAWCH_ADR 84
#define THROTTLECH_ADR 88
#define MODECH_ADR 92
#define AUXCH_ADR 96
#define ROLLPIN_ADR 100
#define PITCHPIN_ADR 104
#define YAWPIN_ADR 108
#define THROTTLEPIN_ADR 112
#define MODEPIN_ADR 116
#define AUXPIN_ADR 120*/

// Motor control variables
#define MINCOMMAND 1000
#define MIDCOMMAND 1500
#define MAXCOMMAND 2000
#define MINDELTA 200
#define MINCHECK MINCOMMAND + 100
#define MAXCHECK MAXCOMMAND - 100
#define MINTHROTTLE MINCOMMAND + 50
#define FRONTMOTORPIN 8
#define REARMOTORPIN 9
#define RIGHTMOTORPIN 10
#define LEFTMOTORPIN 11
#define FRONT 0
#define REAR 1
#define RIGHT 2
#define LEFT 3
#define LASTMOTOR 4
ServoTimer2 frontMotor;
ServoTimer2 rearMotor;
ServoTimer2 rightMotor;
ServoTimer2 leftMotor;
int motorCommand[4] = {1000,1000,1000,1000};
int motorAxisCommand[3] = {0,0,0};
int motor, minCommand = 0;
// If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V 
// Scale gyro output (-465 to +465) to motor commands (1000 to 2000) 
// use y = mx + b 
float mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465)) 
float bMotorRate = 1500;   // b = y1 - m * x1 

// Transmitter variables
#define TIMEOUT 25000
#define THROTTLEPIN 4
#define ROLLPIN 2
#define PITCHPIN 3
#define YAWPIN 6
#define MODEPIN 7
#define AUXPIN 5
#define ROLL 0
#define PITCH 1
#define YAW 2
#define THROTTLE 3
#define MODE 4
#define AUX 5
#define LASTCHANNEL 6
volatile int transmitterData[6];
int orderCh[6] = {ROLL,AUX,MODE,PITCH,THROTTLE,YAW}; // order the channels are output from receiver
int xmitCh[6] = {2,5,7,3,4,6}; // digital pin assignments for each channel
int transmitterCommand[4] = {1500,1500,1500,1000};
int transmitterZero[3] = {1500,1500,1500};
int channel = 0;
// Controls the strength of the commands sent from the transmitter
// xmitFactor ranges from 0.01 - 1.0 (0.01 = weakest, 1.0 - strongest)
float xmitFactor; // Read in from EEPROM

// These A/D values depend on how well the sensors are mounted
// change these values to your unique configuration
// #define XMIN 405
// #define XMAX 607
// #define YMIN 409
// #define YMAX 618
#define ZMIN 403
#define ZMAX 611
#define ZAXIS 2
#define ZEROLIMIT 2
int axis;

// Accelerometer setup
int accelData[3] = {0,0,0};
int accelZero[3] = {0,0,0};
int accelADC[3] = {0,0,0};

// Auto level setup
int levelAdjust[2] = {0,0};
int levelLimit; // Read in from EEPROM
int levelInterval; // Read in from EEPROM

// Gyro setup
int gyroData[3] = {0,0,0};
int gyroZero[3] = {0,0,0};
int gyroADC[3] = {0,0,0};

// Complementary roll/pitch angle
float flightAngle[2] = {0,0};
float filterTerm[2] = {0,0};
float timeConstant; // Read in from EEPROM

// Calibration parameters
#define FINDZERO 50
int findZero[FINDZERO];

// Low pass filter parameters
#define GYRO 0
#define ACCEL 1
float smoothFactor[2]; // Read in from EEPROM

// PID Values
#define LASTAXIS 3
#define LEVELROLL 3
#define LEVELPITCH 4
#define LASTLEVELAXIS 5
struct PIDdata {
  float P, I, D;
  float lastPosition;
  float integratedError;
} PID[5];
float windupGuard; // Read in from EEPROM

// Communication
char queryType = 'X';
byte tlmType = 0;
char string[32];
byte armed = 0;
byte safetyCheck = 0;
volatile byte update = 0;

// Interrupt handler variables
volatile byte timeSlot;

// Timing
long previousTime = 0;
long currentTime = 0;
long deltaTime = 0;
long enableAcc = 0;
float dt = 0;

// ******************** Setup AeroQuadAero ********************
void setup() {
  Serial.begin(BAUD);
  analogReference(EXTERNAL); // Current external ref is connected to 3.3V
  pinMode (LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  
  // Configure motors
  configureMotors();
  commandAllMotors(MINCOMMAND);

  // Read user values from EEPROM
  readEEPROM();
  
  // Setup interrupt to trigger on pin D2
  configureTransmitter();
  
  // Wait for battery to be plugged in before ESC's are armed
  //armESC(3000);
  
  // Calibrate sensors
  zeroGyros();
  
  // Complementary filter setup
  configureFilter(timeConstant);
  
  previousTime = millis();
  digitalWrite(LEDPIN, HIGH);
  safetyCheck = 0;
}

// ******************** Main AeroQuad Loop ********************
void loop () {
  // Measure loop rate
  currentTime = millis();
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;
  enableAcc++;
  
  // Send configuration commands from transmitter
  if (transmitterData[THROTTLE] < 1010) {
    zeroIntegralError();
    // Disarm motors (throttle down, yaw left)
    //if (yaw < MINCHECK && armed == 1) {
    if (transmitterData[YAW] > MAXCHECK && armed == 1) {
      armed = 0;
      commandAllMotors(MINCOMMAND);
    }    
    // Zero sensors (throttle down, yaw left, roll left)
    //if (yaw < MINCHECK && roll > MAXCHECK) {
    if (transmitterData[YAW] > MAXCHECK && transmitterData[ROLL] < MINCHECK) {
      zeroGyros();
      zeroAccelerometers();
      zeroIntegralError();
      pulseMotors(3);
    }   
    // Arm motors (throttle down, yaw right)  
    //if (yaw > MAXCHECK && armed == 0) {
    if (transmitterData[YAW] < MINCHECK && armed == 0 && safetyCheck == 1) {
      armed = 1;
      zeroIntegralError();
      minCommand = MINTHROTTLE;
    }
    if (transmitterData[YAW] > MINCHECK) safetyCheck = 1; 
  }
  else if (transmitterData[THROTTLE] > (MIDCOMMAND - MINDELTA)) minCommand = transmitterData[THROTTLE] - MINDELTA;
  if (transmitterData[THROTTLE] < MINTHROTTLE) minCommand = MINTHROTTLE;

  // Read Sensors
  // Apply low pass filter to sensor values and center around zero
  // Did not convert to engineering units, since will apply P gain anyway
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    gyroADC[axis] = analogRead(gyroChannel[axis]) - gyroZero[axis];
    accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
  }
  
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    gyroData[axis] = smooth(gyroADC[axis], gyroData[axis], smoothFactor[GYRO]);
    accelData[axis] = smooth(accelADC[axis], accelData[axis], smoothFactor[ACCEL]);
  }
  
  dt = deltaTime / 1000.0;
  for (axis = ROLL; axis < YAW; axis++)
    flightAngle[axis] = filterData(flightAngle[axis], gyroADC[axis], accelADC[axis], filterTerm, dt);
  
  if (transmitterData[MODE] < 1500) {
    // Acrobatic Mode
    levelAdjust[ROLL] = 0;
    levelAdjust[PITCH] = 0;
  }
  else {
    // Stable Mode
    if ((enableAcc % levelInterval) == 0) {
      for (axis = ROLL; axis < YAW; axis++)
        levelAdjust[axis] = limitRange(updatePID(0, flightAngle[axis], &PID[LEVELROLL + axis]), -levelLimit, levelLimit);
    }
  }
  
  // Transmitter Commands
  // Calculations are performed in Transmitter.pde when receiver channels are read
  // Commands calculated are transmitterCommand[ROLL], transmitterCommand[PITCH], transmitterCommand[YAW]
  // Saves processing time, since values only change when reading receiver
  
  // Update PID
  motorAxisCommand[ROLL] = updatePID(transmitterCommand[ROLL] - levelAdjust[ROLL], (gyroData[ROLL] * mMotorRate) + bMotorRate, &PID[ROLL]);
  motorAxisCommand[PITCH] = updatePID(transmitterCommand[PITCH] + levelAdjust[PITCH], (gyroData[PITCH] * mMotorRate) + bMotorRate, &PID[PITCH]);
  motorAxisCommand[YAW] = updatePID(transmitterCommand[YAW], (gyroData[YAW] * mMotorRate) + bMotorRate, &PID[YAW]);
    
  // Calculate motor commands
  if (armed && safetyCheck) {
    // Original Orientation
    motorCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    // New Orientation
    /*motorCommand[FRONT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[PITCH] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[REAR] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[PITCH] + motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[RIGHT] = limitRange(transmitterCommand[THROTTLE] - motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);
    motorCommand[LEFT] = limitRange(transmitterCommand[THROTTLE] + motorAxisCommand[ROLL] - motorAxisCommand[YAW], minCommand, MAXCOMMAND);*/
  }
  else {
    for (motor = FRONT; motor < LASTMOTOR; motor++)
      motorCommand[motor] = MINCOMMAND;
  }
  
  // Command motors
  commandMotors();
  
  // Check for remote commands and send requested telemetry
  readSerialCommand();
  if (update) sendSerialTelemetry();
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

// Utilities for writing and reading from the EEPROM
float readFloat(int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatOut;
  
  for (int i = 0; i < 4; i++) 
    floatOut.floatByte[i] = EEPROM.read(address + i);
  return floatOut.floatVal;
}

void writeFloat(float value, int address) {
  union floatStore {
    byte floatByte[4];
    float floatVal;
  } floatIn;
  
  floatIn.floatVal = value;
  for (int i = 0; i < 4; i++) 
    EEPROM.write(address + i, floatIn.floatByte[i]);
}

void readEEPROM() {
  for (axis = ROLL; axis < YAW; axis++) {
    PID[axis].P = readFloat(PGAIN_ADR);
    PID[axis].I = readFloat(IGAIN_ADR);
    PID[axis].D = readFloat(DGAIN_ADR);
    PID[axis].lastPosition = 0;
    PID[axis].integratedError = 0;
  }
  
  PID[YAW].P = readFloat(YAW_PGAIN_ADR);
  PID[YAW].I = readFloat(YAW_IGAIN_ADR);
  PID[YAW].D = readFloat(YAW_DGAIN_ADR);
  PID[YAW].lastPosition = 0;
  PID[YAW].integratedError = 0;
  
  for (axis = LEVELROLL; axis < (LASTLEVELAXIS); axis++) {
    PID[axis].P = readFloat(LEVEL_PGAIN_ADR);
    PID[axis].I = readFloat(LEVEL_IGAIN_ADR);
    PID[axis].D = readFloat(LEVEL_DGAIN_ADR);
    PID[axis].lastPosition = 0;
    PID[axis].integratedError = 0;
  }  
  
  windupGuard = readFloat(WINDUPGUARD_ADR);
  levelLimit = readFloat(LEVELLIMIT_ADR);
  levelInterval = readFloat(LEVELINTERVAL_ADR);
  xmitFactor = readFloat(XMITFACTOR_ADR);
  smoothFactor[GYRO] = readFloat(GYROSMOOTH_ADR);
  smoothFactor[ACCEL] = readFloat(ACCSMOOTH_ADR);
  accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
  accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
  accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  timeConstant = readFloat(FILTERTERM_ADR);
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

void configureFilter(float timeConstant) {
  filterTerm[0] = timeConstant / (timeConstant + 0.010); //10ms = ESC update rate
  filterTerm[1] = 1 - filterTerm[0];
}

float filterData(float previousAngle, int gyroADC, int accelADC, float *filterTerm, float dt) {
  // For Sparkfun 5DOF IMU
  // accelerometerOutput = (N-512)/1024*(double)10.78;
  // gyroOutput = (N-512)/1024*(double)28.783;
  return (filterTerm[0] * (previousAngle + (gyroADC / 1024.0 * 28.783 * dt))) + (filterTerm[1] * (accelADC / 1024.0 * 10.78)) * 57.2957795;
}

int smooth(int currentData, int previousData, float smoothFactor) {
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

void configureMotors() {
  frontMotor.attach(FRONTMOTORPIN);
  rearMotor.attach(REARMOTORPIN);
  rightMotor.attach(RIGHTMOTORPIN);
  leftMotor.attach(LEFTMOTORPIN);
}

void commandMotors() {
  frontMotor.write(motorCommand[FRONT]);
  rearMotor.write(motorCommand[REAR]);
  rightMotor.write(motorCommand[RIGHT]);
  leftMotor.write(motorCommand[LEFT]);
}

// Sends commands to all motors
void commandAllMotors(int motorCommand) {
  frontMotor.write(motorCommand);
  rearMotor.write(motorCommand);
  rightMotor.write(motorCommand);
  leftMotor.write(motorCommand);
}

void pulseMotors(byte quantity) {
  for (byte i=0; i<quantity; i++) {      
    commandAllMotors(MINCOMMAND + 50);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}

/*
void armESC(unsigned long msWaitTime) {
  unsigned long startTime;
  startTime = millis();
  while ((millis() - startTime) < msWaitTime) {
    digitalWrite(FRONTMOTORPIN, HIGH);
    digitalWrite(REARMOTORPIN, HIGH);
    digitalWrite(RIGHTMOTORPIN, HIGH);
    digitalWrite(LEFTMOTORPIN, HIGH);
    delay(1);
    digitalWrite(FRONTMOTORPIN, LOW);
    digitalWrite(REARMOTORPIN, LOW);
    digitalWrite(RIGHTMOTORPIN, LOW);
    digitalWrite(LEFTMOTORPIN, LOW);
    delay(20);
  }
}
*/

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters)
{
  float error;
  float dTerm;

  error = targetPosition - currentPosition;
  
  PIDparameters->integratedError += error;
  if (PIDparameters->integratedError < -windupGuard) PIDparameters->integratedError = -windupGuard;
  else if (PIDparameters->integratedError > windupGuard) PIDparameters->integratedError = windupGuard;
  
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
  PIDparameters->lastPosition = currentPosition;
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

void zeroIntegralError() {
  for (axis = ROLL; axis < LASTLEVELAXIS; axis++)
    PID[axis].integratedError = 0;
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

// Utilities for sensor measurements

int findMode(int *data, int arraySize) {
  // The mode of a set of numbers is the value that occurs most frequently
  boolean done = 0;
  byte i;
  int temp, maxData, frequency, maxFrequency;
  
  // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (i=0; i<(arraySize-1); i++) {
      if (data[i] > data[i+1]) {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }
  
  temp = 0;
  frequency = 0;
  maxFrequency = 0;
  
  // Count number of times a value occurs in sorted array
  for (i=0; i<arraySize; i++) {
    if (data[i] > temp) {
      frequency = 0;
      temp = data[i];
      frequency++;
    } else if (data[i] == temp) frequency++;
    if (frequency > maxFrequency) {
      maxFrequency = frequency;
      maxData = data[i];
    }
  }
  return maxData;
}

// Allows user to zero gyros on command
void zeroGyros() {
  for (axis = ROLL; axis < LASTAXIS; axis++) {
    for (int i=0; i<FINDZERO; i++) findZero[i] = analogRead(gyroChannel[axis]);
    gyroZero[axis] = findMode(findZero, FINDZERO);
  }
}

// Allows user to zero accelerometers on command
void zeroAccelerometers() {
  for (axis = ROLL; axis < YAW; axis++) {
    for (int i=0; i<FINDZERO; i++) findZero[i] = analogRead(accelChannel[axis]);
    accelZero[axis] = findMode(findZero, FINDZERO);
  }
  accelZero[ZAXIS] = ZMAX - ((ZMAX - ZMIN)/2);
  writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
  writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
  writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
}

// Works faster and is smaller than the constrain() function
int limitRange(int data, int minLimit, int maxLimit) {
  if (data < minLimit) return minLimit;
  else if (data > maxLimit) return maxLimit;
  else return data;
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

void readSerialCommand() {
  // Check for serial message
  if (Serial.available()) {
    digitalWrite(LEDPIN, LOW);
    queryType = Serial.read();
    if (queryType == 'C') {
      // Configure user defined values
      PID[ROLL].P = readFloatSerial();
      PID[ROLL].I = readFloatSerial();
      PID[ROLL].D = readFloatSerial();
      PID[ROLL].lastPosition = 0;
      PID[ROLL].integratedError = 0;
      PID[PITCH] = PID[ROLL];
      PID[LEVELROLL].P = readFloatSerial();
      PID[LEVELROLL].I = readFloatSerial();
      PID[LEVELROLL].D = readFloatSerial();
      PID[LEVELROLL].lastPosition = 0;
      PID[LEVELROLL].integratedError = 0;
      PID[LEVELPITCH] = PID[LEVELROLL];
      PID[YAW].P = readFloatSerial();
      PID[YAW].I = readFloatSerial();
      PID[YAW].D = readFloatSerial();
      PID[YAW].lastPosition = 0;
      PID[YAW].integratedError = 0;
      windupGuard = readFloatSerial();
      levelLimit = readFloatSerial();
      levelInterval = readFloatSerial();
      xmitFactor = readFloatSerial();
      smoothFactor[GYRO] = readFloatSerial();
      smoothFactor[ACCEL] = readFloatSerial();
      timeConstant = readFloatSerial();
    }
    else if (queryType == 'W') {
      writeFloat(PID[ROLL].P, PGAIN_ADR);
      writeFloat(PID[ROLL].I, IGAIN_ADR);
      writeFloat(PID[ROLL].D, DGAIN_ADR);
      writeFloat(PID[LEVELROLL].P, LEVEL_PGAIN_ADR);
      writeFloat(PID[LEVELROLL].I, LEVEL_IGAIN_ADR);
      writeFloat(PID[LEVELROLL].D, LEVEL_DGAIN_ADR);
      writeFloat(PID[YAW].P, YAW_PGAIN_ADR);
      writeFloat(PID[YAW].I, YAW_IGAIN_ADR);
      writeFloat(PID[YAW].D, YAW_DGAIN_ADR);
      writeFloat(windupGuard, WINDUPGUARD_ADR);  
      writeFloat(levelLimit, LEVELLIMIT_ADR);   
      writeFloat(levelInterval, LEVELINTERVAL_ADR); 
      writeFloat(xmitFactor, XMITFACTOR_ADR);
      writeFloat(smoothFactor[GYRO], GYROSMOOTH_ADR);
      writeFloat(smoothFactor[ACCEL], ACCSMOOTH_ADR);
      writeFloat(timeConstant, FILTERTERM_ADR);
      zeroIntegralError();
      
      // Complementary filter setup
      configureFilter(timeConstant);
    }
    /*else if (queryType =='M') { // remotely configure receiver order
      for (channel = ROLL; channel < LASTCHANNEL; channel++) {
        orderCh[channel] = readFloatSerial();
        writeFloat(orderCh[channel], ROLLCH_ADR + (channel * 4));
      }
      for (channel = ROLL; channel < LASTCHANNEL; channel++) {
        xmitCh[channel] = readFloatSerial();
        writeFloat(xmitCh[channel], ROLLPIN_ADR + (channel * 4));
      }
    }*/
  digitalWrite(LEDPIN, HIGH);
  }
}

// Used to read floating point values from the serial port
float readFloatSerial() {
  byte index = 0;
  byte timeout = 0;
  char data[128] = "";

  do {
    if (Serial.available() == 0) {
      delay(10);
      timeout++;
    }
    else {
      data[index] = Serial.read();
      timeout = 0;
      index++;
    }
  }  while ((data[limitRange(index-1, 0, 128)] != ';') && (timeout < 5) && (index < 128));
  return atof(data);
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

void sendSerialTelemetry() {
  update = 0;
  switch (queryType) {
  case 'X': // send no debug messages
    break;
  case 'A': // send all data
    Serial.print(deltaTime);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroData[axis]);
      comma();
    }
    Serial.print(transmitterData[THROTTLE]);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(motorAxisCommand[axis]);
      comma();
    }
    for (motor = FRONT; motor < LASTMOTOR; motor++) {
      Serial.print(motorCommand[motor]);
      comma();
    }
    Serial.print(armed, BIN);
    comma();
    Serial.println(transmitterData[MODE]);
    break;
  case 'S': // send sensor data
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(gyroADC[axis]);
      comma();
    }
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(accelADC[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(flightAngle[ROLL]);
    comma();
    Serial.print(flightAngle[PITCH]);
    Serial.println();
    break;
  case 'D': // raw read sensor data
    Serial.print(analogRead(ROLLRATEPIN));
    comma();
    Serial.print(analogRead(PITCHRATEPIN));
    comma();
    Serial.print(analogRead(YAWRATEPIN));
    comma();
    Serial.print(analogRead(ROLLACCELPIN));
    comma();
    Serial.print(analogRead(PITCHACCELPIN));
    comma();
    Serial.println(analogRead(ZACCELPIN));
    break;
  case 'U': // send user defined values
    //Serial.print(dtostrf(PID[ROLL].P, 1, 2, string));
    Serial.print(PID[ROLL].P);
    comma();
    Serial.print(PID[ROLL].I);
    comma();
    Serial.print(PID[ROLL].D);
    comma();
    Serial.print(PID[LEVELROLL].P);
    comma();
    Serial.print(PID[LEVELROLL].I);
    comma();
    Serial.print(PID[LEVELROLL].D);
    comma();
    Serial.print(PID[YAW].P);
    comma();
    Serial.print(PID[YAW].I);
    comma();
    Serial.print(PID[YAW].D);
    comma();
    Serial.print(windupGuard);
    comma();
    Serial.print(levelLimit);
    comma();
    Serial.print(levelInterval);
    comma();
    Serial.print(xmitFactor);
    comma();
    Serial.print(smoothFactor[GYRO]);
    comma();
    Serial.print(smoothFactor[ACCEL]);
    comma();
    Serial.print(timeConstant);
    Serial.println(); // will probably add more responses in the future
    queryType = 'X';
    break;
   case 'T': // read processed transmitter values
    Serial.print(xmitFactor);
    comma();
    for (axis = ROLL; axis < LASTAXIS; axis++) {
      Serial.print(transmitterCommand[axis]);
      comma();
    }
    for (axis = ROLL; axis < YAW; axis++) {
      Serial.print(levelAdjust[axis]);
      comma();
    }
    Serial.print(motorAxisCommand[ROLL]);
    comma();
    Serial.print(motorAxisCommand[PITCH]);
    comma();
    Serial.println(motorAxisCommand[YAW]);
    break;
  case 'R': // send receiver values
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(transmitterData[channel]);
      comma();
    }
    Serial.println(transmitterData[AUX]);
    break;
  /*case 'N': // send receiver channel order
    for (channel = ROLL; channel < LASTCHANNEL; channel++) {
      Serial.print(orderCh[channel]);
      comma();
    }
    for (channel = ROLL; channel < AUX; channel++) {
      Serial.print(xmitCh[channel]);
      comma();
    }
    Serial.println(xmitCh[AUX]);
    queryType = 'X';
    break;*/
  }
}

void comma() {
  Serial.print(',');
}

/*
  AeroQuad v1.0 - March 2009
  www.AeroQuad.info
  Copyright (c) 2009 Ted Carancho.  All rights reserved.
  An Arduino based quadrocopter using the Sparkfun 5DOF IMU and IDG300 Dual Axis Gyro
  This version will be able to use gyros for stability (acrobatic mode) or accelerometers (experimental stable mode).
 
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

// Interrupts are generated every 20ms because
// Roll pin from transmitter triggers interrupt (50MHz, PWM)
// Modulo division by 5 causes each time slot to happen every 100ms

void configureTransmitter() {
  attachInterrupt(0, readTransmitter, RISING);
}

void readTransmitter() {
  if (((timeSlot += 1) % 5) == 0) {
    // Read Receiver
    // This has been tested with AR6100 and AR6200 Spektrum receivers
    for (channel = 0; channel < 6; channel++)
      transmitterData[orderCh[channel]] = pulseIn(xmitCh[channel], HIGH, TIMEOUT);
    // Calculate Commanded Angles
    // Reduce transmitter commands using xmitFactor and center around 1500
    // rollCommand, pitchCommand and yawCommand used in main loop of AeroQuad.pde to control quad
    for (axis = ROLL; axis < LASTAXIS; axis++)
      transmitterCommand[axis] = ((transmitterData[axis] - transmitterZero[axis]) * xmitFactor) + transmitterZero[axis];
    transmitterCommand[THROTTLE] = transmitterData[THROTTLE];
  }
  if ((timeSlot % 5) == 2) update = 1;
}

int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

