#ifndef _AEROQUAD_I2C_DEVICE_H_
#define _AEROQUAD_I2C_DEVICE_H_

#include <Wire.h>

extern "C" {
	#include <math.h>
	#include "WConstants.h"
}

class I2C_Device 
{
public:
	I2C_Device();
	void InitWireLib(byte initialiseWireLib);
	void I2cWriteRegister(byte deviceAddress, byte registerAddress, byte value);
	byte I2cReadMemory(byte deviceAddress, byte memoryAddress, byte numberOfBytes, byte _buffer[]);
	byte I2cDetectDevice(byte initialiseWireLib, byte deviceAddress);
};

#endif // _AEROQUAD_I2C_DEVICE_H_

