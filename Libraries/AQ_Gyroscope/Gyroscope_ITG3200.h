/**
	Gyroscope_ITG3200.h (ITG3200, I2C 3-axis gyroscope sensor) library
	by 2010 Ivan Todorovic
*/

#ifndef _GYROSCOPE_ITG3200_H_
#define _GYROSCOPE_ITG3200_H_

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
	void BootCalibrate();
public:
	float X, Y, Z;
	Gyroscope_ITG3200();
	void Init(byte initialiseWireLib = 0);
	void Measure();
	byte DetectPresence(byte initialiseWireLib = 0);
};

#endif	// #ifndef _GYROSCOPE_ITG3200_H_

