/*
  AeroQuad v3.0 - May 2011
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

// parts of the init sequence were taken from AP_InertialSensor_MPU6000.h

#ifndef _AEROQUAD_PLATFORM_MPU6000_H_
#define _AEROQUAD_PLATFORM_MPU6000_H_

// I2C support for MPU6000/6050 is not tested yet
//#define MPU6000_I2C

#include "Arduino.h"
#include <SensorsStatus.h>


// MPU 6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74


// Configuration bits
#define BIT_SLEEP				0x40
#define BIT_H_RESET				0x80
#define BITS_CLKSEL				0x07
#define MPU_CLK_SEL_PLLGYROX	0x01
#define MPU_CLK_SEL_PLLGYROZ	0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_FS_250DPS          0x00
#define BITS_FS_500DPS          0x08
#define BITS_FS_1000DPS         0x10
#define BITS_FS_2000DPS         0x18
#define BITS_FS_MASK            0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR    0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS          0x10
#define BIT_INT_STATUS_DATA		0x01


typedef struct {
	short x;
	short y;
	short z;
} tAxis;

union uMPU6000 {
	unsigned char rawByte[];
	unsigned short rawWord[];
	struct {
		tAxis accel;
		short temperature;
		tAxis gyro;
	} data;
} MPU6000;


#ifdef MPU6000_I2C
	#define MPU6000_I2C_ADDRESS 0x68
#else
	#include <HardwareSPIExt.h>
	HardwareSPIExt spiMPU6000(4);
#endif

void MPU6000_SpiLowSpeed()
{
#ifndef MPU6000_I2C
	spiMPU6000.begin(SPI_562_500KHZ, MSBFIRST, 3);
#endif
}

void MPU6000_SpiHighSpeed()
{
#ifndef MPU6000_I2C
	spiMPU6000.end();
    spiMPU6000.begin(SPI_9MHZ, MSBFIRST, 3);
#endif
}

void MPU6000_WriteReg(int addr, byte data)
{
#ifdef MPU6000_I2C
	updateRegisterI2C(MPU6000_I2C_ADDRESS, addr, data);
#else
	spiMPU6000.Write(addr, data);
#endif
    delay(1);
}

byte MPU6000_ReadReg(int addr)
{
#ifdef MPU6000_I2C
	sendByteI2C(MPU6000_I2C_ADDRESS, addr);
	byte data = readByteI2C(MPU6000_I2C_ADDRESS);
#else
	byte data = spiMPU6000.Read(addr);
#endif
    delay(1);
    return data;
}

void initializeMPU6000Sensors()
{
#ifdef DEBUG_INIT
	Serial.println("initializeMPU6000Sensors");
#endif
	MPU6000_SpiLowSpeed();

	unsigned char val;

	val = MPU6000_ReadReg(MPUREG_WHOAMI);
	if((val&0x7E) == 0x68) {
		vehicleState |= GYRO_DETECTED;
		vehicleState |= ACCEL_DETECTED;
#ifdef DEBUG_INIT
		Serial.println("MPU6000 found");
#endif
	} else {
#ifdef DEBUG_INIT
		Serial.println("MPU6000 not found");
#endif
		return;
	}

    // Chip reset
    MPU6000_WriteReg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);  // Startup time delay

#ifndef MPU6000_I2C
    // Disable I2C bus
    MPU6000_WriteReg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
#endif

    // Wake Up device and select GyroZ clock (better performance)
    MPU6000_WriteReg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    MPU6000_WriteReg(MPUREG_PWR_MGMT_2, 0);


	// SAMPLE RATE
    MPU6000_WriteReg(MPUREG_SMPLRT_DIV,0x00);     // Sample rate = 1kHz

	// FS & DLPF   FS=1000º/s, DLPF = 42Hz (low pass filter)
    MPU6000_WriteReg(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
    MPU6000_WriteReg(MPUREG_GYRO_CONFIG,BITS_FS_1000DPS);  // Gyro scale 1000º/s
    //Serial.print("MPUREG_GYRO_CONFIG read 0x");
    //Serial.println(MPU6000_ReadReg(MPUREG_GYRO_CONFIG), 16);

    MPU6000_WriteReg(MPUREG_ACCEL_CONFIG,0x08);   // Accel scale +-4g (4096LSB/g)

    // INT CFG => Interrupt on Data Ready
    //MPU6000_WriteReg(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);        // INT: Raw data ready
    //MPU6000_WriteReg(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read

    // switch to high clock rate
    MPU6000_SpiHighSpeed();
};


void MPU6000SwapData(unsigned char *data, int datalen)
{
	datalen /= 2;
	while(datalen--) {
		unsigned char t = data[0];
		data[0] = data[1];
		data[1] = t;
		data += 2;
	}
}

void readMPU6000Sensors()
{
#ifdef MPU6000_I2C
    Wire.requestFrom(MPU6000_I2C_ADDRESS, sizeof(MPU6000));
    for(byte i=0; i<sizeof(MPU6000)/2; i++) {
    	MPU6000.rawWord[i] = readWordI2C();
    }
#else
	spiMPU6000.Read(MPUREG_ACCEL_XOUT_H, MPU6000.rawByte, sizeof(MPU6000));
	MPU6000SwapData(MPU6000.rawByte, sizeof(MPU6000));
#endif
}

#endif
