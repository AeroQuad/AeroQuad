/*
  AeroQuad v2.0 - July 2010
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

struct PIDdata {
  float P, I, D;
  float lastPosition;
  float integratedError;
} PID[8];
float windupGuard; // Read in from EEPROM

// Modified from http://www.arduino.cc/playground/Main/BarebonesPIDForEspresso
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters) {
  float error;
  float dTerm;

  error = targetPosition - currentPosition;
  
  PIDparameters->integratedError += error * G_Dt;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -windupGuard, windupGuard);
  
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition);
  PIDparameters->lastPosition = currentPosition;
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

void zeroIntegralError() {
  for (axis = ROLL; axis < LASTLEVELAXIS; axis++)
    PID[axis].integratedError = 0;
}

// Adapted from control method by Jose Julio
float updatePIDangle(float targetPosition, float currentPosition, float gyroData, int receiverData, struct PIDdata *PIDparameters) {
  float error;
  float dTerm;
  float errorRollRate;

  error = targetPosition - currentPosition;
  error = constrain(error, -25, 25);
  
  PIDparameters->integratedError += error * G_Dt;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -windupGuard, windupGuard);
  
 // errorRollRate = ((receiverData - 1500) >> 1) - gyroData;
  //dTerm = -gyroData;
  //PIDparameters->lastPosition = targetPosition;
  
  //Serial.print(error);comma();Serial.print(PIDparameters->integratedError);comma();Serial.print(dTerm);comma();Serial.print(errorRollRate);Serial.println();
  //Serial.print(errorRollRate);comma();Serial.println(errorRollRate * 0.4);
  
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + (gyroData * PIDparameters->D);// * (errorRollRate * 0.4);
}

