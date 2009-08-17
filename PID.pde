/*
  AeroQuad v1.2 - August 2009
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
