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


#include <GyroIXZ500.h>

GyroIXZ500::GyroIXZ500() 
{
  gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
  gyroScaleFactor = 0.4;         // IDG/IXZ500 sensitivity = 2mV/(deg/sec)  2.0mV/Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
}
  
void GyroIXZ500::initialize(void) 
{
  analogReference(EXTERNAL);
  // Configure gyro auto zero pins
  pinMode (AZPIN, OUTPUT);
  digitalWrite(AZPIN, LOW);
  delay(1);

  // rollChannel = 4
  // pitchChannel = 3
  // yawChannel = 5
  this->_initialize(4,3,5);
  smoothFactor = readFloat(GYROSMOOTH_ADR);
}
  
void GyroIXZ500::measure(void) 
{
  currentTime = micros();
  for (byte axis = ROLL; axis < LASTAXIS; axis++) {
    gyroADC[axis] = gyroZero[axis] - analogRead(gyroChannel[axis]);
    gyroData[axis] = filterSmooth(gyroADC[axis], gyroData[axis], smoothFactor);
  }
  previousTime = currentTime;
}

const int GyroIXZ500::getFlightData(byte axis) 
{
  return getRaw(axis);
}
  
void GyroIXZ500::calibrate() 
{
  autoZero();
  writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
}
  
void GyroIXZ500::autoZero() 
{
  int findZero[FINDZERO];
  digitalWrite(AZPIN, HIGH);
  delayMicroseconds(750);
  digitalWrite(AZPIN, LOW);
  delay(8);

  for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
  {
    for (int i=0; i<FINDZERO; i++)
	{
      findZero[i] = analogRead(gyroChannel[calAxis]);
	}
    gyroZero[calAxis] = findModeInt(findZero, FINDZERO);
  }
}
