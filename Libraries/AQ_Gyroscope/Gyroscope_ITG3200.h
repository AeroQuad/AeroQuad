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

#ifndef _AEROQUAD_GYROSCOPE_ITG3200_H_
#define _AEROQUAD_GYROSCOPE_ITG3200_H_

#include "I2C_Device.h"

#define ITG3200_ADDRESS			0x69
#define ITG3200_MEMORY_ADDRESS		0x1D
#define ITG3200_BUFFER_SIZE		6
#define ITG3200_RESET_ADDRESS		0x3E
#define ITG3200_RESET_VALUE		0x80
#define ITG3200_LOW_PASS_FILTER_ADDR	0x16
#define ITG3200_LOW_PASS_FILTER_VALUE	0x1D	// 10Hz low pass filter
#define ITG3200_OSCILLATOR_ADDR		0x3E
#define ITG3200_OSCILLATOR_VALUE	0x01	// use X gyro oscillator
#define ITG3200_SCALE_TO_RADIANS	823.626831 // 14.375 LSBs per °/sec, / Pi / 180

class Gyroscope_ITG3200 : I2C_Device
{
private:
	byte buffer[ITG3200_BUFFER_SIZE];
  int data[3];
  
public:
	Gyroscope_ITG3200();

	void initialize(byte initializeWireLib = 0);
	void measure(void);
	void calibrate(void);
	byte DetectPresence(byte initializeWireLib = 0);
};

#endif	// #ifndef _GYROSCOPE_ITG3200_H_

