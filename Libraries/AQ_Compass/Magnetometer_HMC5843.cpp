/*
  AeroQuad v3.0 - April 2011
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


#include "Magnetometer_HMC5843.h"

#include <Device_I2C.h>
#include <Axis.h>
#include <AQMath.h>
#define COMPASS_ADDRESS 0x1E

Magnetometer_HMC5843::Magnetometer_HMC5843() {
}

void Magnetometer_HMC5843::initialize(float dcm[]) {
  
  
  byte numAttempts = 0;
  bool success = false;
    
  delay(10);    // Power up delay **
    
  firstPassMagBias = 1;
          
  magCalibration[XAXIS] = 1.0;
  magCalibration[YAXIS] = 1.0;
  magCalibration[ZAXIS] = 1.0;
    
  magBias[XAXIS] = 0.0;
  magBias[YAXIS] = 0.0;
  magBias[ZAXIS] = 0.0;
    
  while (success == false && numAttempts < 5 ) {
    
    numAttempts++;
   
    updateRegisterI2C(COMPASS_ADDRESS, 0x00, 0x11);  // Set positive bias configuration for sensor calibraiton
    delay(50);
   
    updateRegisterI2C(COMPASS_ADDRESS, 0x01, 0x20); // Set +/- 1G gain
    delay(10);

    updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x01);  // Perform single conversion
    delay(10);
   
    measure(0.0, 0.0, dcm, 0);                    // Read calibration data
    delay(10);
   
    if ( fabs(magFieldBodyRaw[XAXIS]) > 500.0 && fabs(magFieldBodyRaw[XAXIS]) < 1000.0 && \
         fabs(magFieldBodyRaw[YAXIS]) > 500.0 && fabs(magFieldBodyRaw[YAXIS]) < 1000.0 && \
         fabs(magFieldBodyRaw[ZAXIS]) > 500.0 && fabs(magFieldBodyRaw[ZAXIS]) < 1000.0) {
      magCalibration[XAXIS] = fabs(715.0 / magFieldBodyRaw[XAXIS]);
      magCalibration[YAXIS] = fabs(715.0 / magFieldBodyRaw[YAXIS]);
      magCalibration[ZAXIS] = fabs(715.0 / magFieldBodyRaw[ZAXIS]);
    
      success = true;
    }
   
    updateRegisterI2C(COMPASS_ADDRESS, 0x00, 0x10);  // Set 10hz update rate and normal operaiton
    delay(50);

    updateRegisterI2C(COMPASS_ADDRESS, 0x02, 0x00); // Continuous Update mode
    delay(50);                                     // Mode change delay (1/Update Rate) **
  }

  measure(0.0, 0.0, dcm, 0);                     // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
}

void Magnetometer_HMC5843::measure(float roll, float pitch, float dcm[], byte useMagBias) {
  float magX;
  float magY;
  float tmp;
  float magFieldBody[3];
    
  sendByteI2C(COMPASS_ADDRESS, 0x03);
  Wire.requestFrom(COMPASS_ADDRESS, 6);

  magFieldBodyRaw[XAXIS] =  ((Wire.receive() << 8) | Wire.receive()) * magCalibration[XAXIS];
  magFieldBodyRaw[YAXIS] = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[YAXIS];
  magFieldBodyRaw[ZAXIS] = -((Wire.receive() << 8) | Wire.receive()) * magCalibration[ZAXIS];

  Wire.endTransmission();
    
  magFieldBody[XAXIS] = magFieldBodyRaw[XAXIS] * magScale[XAXIS] + magOffset[XAXIS] - magBias[XAXIS];
  magFieldBody[YAXIS] = magFieldBodyRaw[YAXIS] * magScale[YAXIS] + magOffset[YAXIS] - magBias[YAXIS];
  magFieldBody[ZAXIS] = magFieldBodyRaw[ZAXIS] * magScale[ZAXIS] + magOffset[ZAXIS] - magBias[ZAXIS];
    
  if (useMagBias == 1)
    magBiasCalculation(dcm, magFieldBody);
    
  float cosRoll =  cos(roll);
  float sinRoll =  sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  magX = magFieldBody[XAXIS] * cosPitch + \
         magFieldBody[YAXIS] * sinRoll * sinPitch + \
         magFieldBody[ZAXIS] * cosRoll * sinPitch;
           
  magY = magFieldBody[YAXIS] * cosRoll - \
         magFieldBody[ZAXIS] * sinRoll;

  tmp  = sqrt(magX * magX + magY * magY);
    
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

void Magnetometer_HMC5843::magBiasCalculation(float dcmMatrix[], float magFieldBody[])
{
  int   i, j ;
  float biasSum[3] ;
  float tempMatrix[3];
    
  // Compute magnetic field of the earth
  matrixMultiply(3, 3, 1, magFieldEarth, dcmMatrix, magFieldBody);
    
  // First pass thru?
  if (firstPassMagBias == 1)
  {
    setPastValues(dcmMatrix, magFieldBody);  // Yes, set initial values for previous values
    firstPassMagBias = 0;                    // Clear first pass flag
  }
    
  // Compute the bias in the magnetometer:
  vectorAdd(3, biasSum , magFieldBody, magFieldBodyPrevious) ;
    
  matrixMultiply(1, 3, 3, tempMatrix, magFieldEarthPrevious, dcmMatrix);
 
  vectorSubtract(3, biasSum, biasSum, tempMatrix);
    
  matrixMultiply(1, 3, 3, tempMatrix, magFieldEarth, dcmMatrixPrevious);
    
  vectorSubtract(3, biasSum, biasSum, tempMatrix) ;
    
  for ( i = 0 ; i < 3 ; i++ )
  {
    if ( abs(biasSum[i] ) < 3 )
    {
      biasSum[i] = 0 ;
    }
  }
   
  vectorAdd (3, magBias, magBias, biasSum);
   
  setPastValues(dcmMatrix, magFieldBody);
}
  
////////////////////////////////////////////////////////////////////////////////
void Magnetometer_HMC5843::setPastValues(float dcmMatrix[], float magFieldBody[])
{
  int i;
   
  for (i = 0 ; i < 3 ; i++)
  {
    magFieldEarthPrevious[i] = magFieldEarth[i];
    magFieldBodyPrevious[i]  = magFieldBody[i];
  }
    
  for (i = 0 ; i < 9 ; i++)
  {
    dcmMatrixPrevious[i] = dcmMatrix[i];
  }
}




