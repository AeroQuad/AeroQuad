/*
  AeroQuad v2.1 - January 2011
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

#include <AccelWii.h>

AccelWii::AccelWii(WiiSensors wiiSensors)
{
  _wiiSensors = &wiiSensors;
  accelScaleFactor = 0;    
}
  
void AccelWii::initialize(void) 
{
  _wiiSensors->initGyroAcc();
  smoothFactor = readFloat(ACCSMOOTH_ADR);
  accelZero[ROLL] = readFloat(LEVELROLLCAL_ADR);
  accelZero[PITCH] = readFloat(LEVELPITCHCAL_ADR);
  accelZero[ZAXIS] = readFloat(LEVELZCAL_ADR);
  accelOneG = readFloat(ACCEL1G_ADR);
}
  
void AccelWii::measure(void) 
{
  currentAccelTime = micros();
  // Actual measurement performed in gyro class
  // We just update the appropriate variables here
  for (byte axis = ROLL; axis < LASTAXIS; axis++) 
  {
    accelADC[axis] = accelZero[axis] - _wiiSensors->NWMP_acc[axis];
    accelData[axis] = filterSmoothWithTime(accelADC[axis], accelData[axis], smoothFactor, ((currentAccelTime - previousAccelTime) / 5000.0));
  }
  previousAccelTime = currentAccelTime;
}
  
const int AccelWii::getFlightData(byte axis) 
{
  return getRaw(axis);
}
 
// Allows user to zero accelerometers on command
void AccelWii::calibrate(void) 
{
  int findZero[FINDZERO];

  for(byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
  {
    for (int i=0; i<FINDZERO; i++)
	{
      _wiiSensors->updateControls();
      findZero[i] = _wiiSensors->NWMP_acc[calAxis];
    }
    accelZero[calAxis] = findModeInt(findZero, FINDZERO);
  }
    
  // store accel value that represents 1g
  accelOneG = getRaw(ZAXIS);
  // replace with estimated Z axis 0g value
  accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
    
  writeFloat(accelOneG, ACCEL1G_ADR);
  writeFloat(accelZero[ROLL], LEVELROLLCAL_ADR);
  writeFloat(accelZero[PITCH], LEVELPITCHCAL_ADR);
  writeFloat(accelZero[ZAXIS], LEVELZCAL_ADR);
}

void AccelWii::calculateAltitude(void) 
{
  currentAccelTime = micros();
  if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500)) 
  {
    rawAltitude += (getZaxis(currentAccelTime,previousAccelTime)) * ((currentAccelTime - previousAccelTime) / 1000000.0);
  }
  previousAccelTime = currentAccelTime;
} 
