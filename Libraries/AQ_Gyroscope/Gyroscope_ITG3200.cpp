/**
	Gyroscope_ITG3200.cpp (ITG3200, I2C 3-axis gyroscope sensor) library
	by Ivan Todorovic
	
	This library is free software: you can redistribute it and/or modify
	it under the terms of the GNU Lesser General Public License as published
	by the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
	GNU Lesser General Public License for more details.
	
	You should have received a copy of the GNU Lesser General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

#include "Gyroscope_ITG3200.h"
#include "

Gyroscope_ITG3200::Gyroscope_ITG3200() {}

void Gyroscope_ITG3200::intialize(byte initializeWireLib) 
{
	InitWireLib(initializeWireLib);
	I2cWriteRegister(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE);
	I2cWriteRegister(ITG3200_ADDRESS, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_VALUE);
	I2cWriteRegister(ITG3200_ADDRESS, ITG3200_OSCILLATOR_ADDR, ITG3200_OSCILLATOR_VALUE);
}

void Gyroscope_ITG3200::measure()
{
	if (I2cReadMemory(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS, ITG3200_BUFFER_SIZE, &buffer[0]) \ 
		== ITG3200_BUFFER_SIZE)  // All bytes received?
	{
		data[0] = (((buffer[0] << 8) | buffer[1]) - zero[0]);
		data[1] = (((buffer[2] << 8) | buffer[3]) - zero[1]);
		data[2] = (((buffer[4] << 8) | buffer[5]) - zero[2]);
    
    for (int i=0; i<3; i++)
      rate[i] = data[i] / ITG3200_SCALE_TO_RADIANS;
	}
}

byte Gyroscope_ITG3200::detectPresence(byte initializeWireLib)
{
	return I2cDetectDevice(initializeWireLib, ITG3200_ADDRESS);
}

void Gyroscope_ITG3200::calibrate() {
  float findZero[FINDZERO];
    
  for (byte calAxis = 0; calAxis < 3; calAxis++) {
    for (int i=0; i<FINDZERO; i++) {
	  measure();
      findZero[i] = data[calAxis];
      delay(measureDelay);
    }
    zero[calAxis] = findMedian(findZero, FINDZERO);
  }
}