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

#include <GyroWii.h>

GyroWii::GyroWii(WiiSensors wiiSensors)
{
  _wiiSensors = &wiiSensors;
  // 0.5mV/Ã‚Âº/s, 0.2mV/ADC step => 0.2/3.33 = around 0.069565217391304
  // @see http://invensense.com/mems/gyro/documents/PS-IDG-0650B-00-05.pdf and
  // @see http://invensense.com/mems/gyro/documents/ps-isz-0650b-00-05.pdf
  gyroFullScaleOutput = 2000;
  gyroScaleFactor = 0.069565217391304;
}
  
void GyroWii::initialize(void) 
{
  _wiiSensors->initGyroAcc(); // defined in DataAquisition.h
  smoothFactor = readFloat(GYROSMOOTH_ADR);
  gyroZero[ROLL] = readFloat(GYRO_ROLL_ZERO_ADR);
  gyroZero[PITCH] = readFloat(GYRO_PITCH_ZERO_ADR);
  gyroZero[ZAXIS] = readFloat(GYRO_YAW_ZERO_ADR);
}
  
void GyroWii::measure(void) 
{
  currentTime = micros();
  _wiiSensors->updateControls(); // defined in DataAcquisition.h
  gyroADC[ROLL] = _wiiSensors->NWMP_gyro[ROLL] - gyroZero[ROLL];
  gyroData[ROLL] = filterSmoothWithTime(gyroADC[ROLL], gyroData[ROLL], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
  gyroADC[PITCH] = gyroZero[PITCH] - _wiiSensors->NWMP_gyro[PITCH];
  gyroData[PITCH] = filterSmoothWithTime(gyroADC[PITCH], gyroData[PITCH], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
  gyroADC[YAW] =  gyroZero[YAW] - _wiiSensors->NWMP_gyro[YAW];
  gyroData[YAW] = filterSmoothWithTime(gyroADC[YAW], gyroData[YAW], smoothFactor, ((currentTime - previousTime) / 5000.0)); //expect 5ms = 5000Ã‚Âµs = (current-previous) / 5000.0 to get around 1
  previousTime = currentTime;
}

const int GyroWii::getFlightData(byte axis) 
{
  return getRaw(axis);
}

void GyroWii::calibrate() 
{
  int findZero[FINDZERO];
  
  for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
  {
    for (int i=0; i<FINDZERO; i++) 
	{
      _wiiSensors->updateControls();
      findZero[i] = _wiiSensors->NWMP_gyro[calAxis];
    }
    gyroZero[calAxis] = findModeInt(findZero, FINDZERO);
  }
  writeFloat(gyroZero[ROLL], GYRO_ROLL_ZERO_ADR);
  writeFloat(gyroZero[PITCH], GYRO_PITCH_ZERO_ADR);
  writeFloat(gyroZero[YAW], GYRO_YAW_ZERO_ADR);
}
