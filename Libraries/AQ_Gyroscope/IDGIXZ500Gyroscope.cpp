/*
  AeroQuad v2.2 - Feburary 2011
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

#include "IDGIXZ500Gyroscope.h"

#include <AQMath.h>

#define AZPIN 12 // Auto zero pin for IDG500 gyros

IDGIXZ500Gyroscope::IDGIXZ500Gyroscope()
{
  _gyroFullScaleOutput = 500.0;   // IDG/IXZ500 full scale output = +/- 500 deg/sec
  _gyroScaleFactor = 0.4;         // IDG/IXZ500 sensitivity = 2mV/(deg/sec)  2.0mV/Âº/s, 0.8mV/ADC step => 0.8/3.33 = 0.4
}
  
void IDGIXZ500Gyroscope::initialize() 
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
}
  
void IDGIXZ500Gyroscope::measure() 
{
  for (byte axis = ROLL; axis < LASTAXIS; axis++) 
  {
    _gyroADC[axis] = _gyroZero[axis] - analogRead(_gyroChannel[axis]);
    _gyroData[axis] = filterSmooth(_gyroADC[axis], _gyroData[axis], _smoothFactor);
  }
}

void IDGIXZ500Gyroscope::calibrate() 
{
  autoZero();
}
  
void IDGIXZ500Gyroscope::autoZero() 
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
      findZero[i] = analogRead(_gyroChannel[calAxis]);
    }
    _gyroZero[calAxis] = findMedianInt(findZero, FINDZERO);
  }
}
