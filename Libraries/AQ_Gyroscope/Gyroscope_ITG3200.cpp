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

Gyroscope_ITG3200::Gyroscope_ITG3200() {}

void Gyroscope_ITG3200::Init(byte initialiseWireLib) 
{
	InitWireLib(initialiseWireLib);
	BootCalibrate();
}

void Gyroscope_ITG3200::BootCalibrate() 
{
	// Reset the device, ste default register values.
	I2cWriteRegister(ITG3200_ADDRESS, ITG3200_RESET_ADDRESS, ITG3200_RESET_VALUE);
	I2cWriteRegister(ITG3200_ADDRESS, ITG3200_LOW_PASS_FILTER_ADDR, ITG3200_LOW_PASS_FILTER_VALUE);
	I2cWriteRegister(ITG3200_ADDRESS, ITG3200_OSCILLATOR_ADDR, ITG3200_OSCILLATOR_VALUE);
}

void Gyroscope_ITG3200::Measure()
{
	if (I2cReadMemory(ITG3200_ADDRESS, ITG3200_MEMORY_ADDRESS, ITG3200_BUFFER_SIZE, &buffer[0]) \ 
		== ITG3200_BUFFER_SIZE)  // All bytes received?
	{
		X = ((buffer[0] << 8) | buffer[1]) / ITG3200_SCALE_TO_RADIANS;
		Y = ((buffer[2] << 8) | buffer[3]) / ITG3200_SCALE_TO_RADIANS;
		Z = ((buffer[4] << 8) | buffer[5]) / ITG3200_SCALE_TO_RADIANS;
	}
}

byte Gyroscope_ITG3200::DetectPresence(byte initialiseWireLib)
{
	return I2cDetectDevice(initialiseWireLib, ITG3200_ADDRESS);
}

