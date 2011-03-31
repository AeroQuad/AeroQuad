/*
	I2C_Device.cpp - Arduino Library for reading I2C sensors
	Code by Ivan Todorovic
	
	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.
*/

#include "I2C_Device.h"

I2C_Device::I2C_Device() {}

void I2C_Device::InitWireLib(byte initialiseWireLib)
{
	if(initialiseWireLib != 0)
	{
		Wire.begin();
		delay(10);
	}
}

void I2C_Device::I2cWriteRegister(byte deviceAddress, byte registerAddress, byte value)
{
	Wire.beginTransmission(deviceAddress);
	Wire.send(registerAddress);
	Wire.send(value);
	Wire.endTransmission();
}

byte I2C_Device::I2cReadMemory(byte deviceAddress, byte memoryAddress, byte numberOfBytes, byte _buffer[])
{
	byte i;
	Wire.beginTransmission(deviceAddress); 
	Wire.send(memoryAddress);
	Wire.endTransmission();

	Wire.requestFrom(deviceAddress, numberOfBytes);
	i = 0; 
	while (Wire.available()) {
		_buffer[i] = Wire.receive();	// receive one byte
		i++;
	}
	Wire.endTransmission();
	return i;	// number of bytes recieved, for error checking
}

byte I2C_Device::I2cDetectDevice(byte initialiseWireLib, byte deviceAddress)
{
	byte i;

	InitWireLib(initialiseWireLib);
	Wire.requestFrom(deviceAddress, (byte)1);
	delay(1);
	if (Wire.available() != 0) // Found device
	{	
		i = Wire.receive();
		return 0;
	}
	else
		return -1;
}

