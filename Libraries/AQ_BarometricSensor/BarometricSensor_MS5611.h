/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

// parts of this code were taken from AN520, an early version of fabio's library and the AQ BMP085 code

#ifndef _AQ_BAROMETRIC_SENSOR_MS5611_
#define _AQ_BAROMETRIC_SENSOR_MS5611_

#include "BarometricSensor.h"
#include "Device_I2C.h"
#include <AQMath.h>


//#define DEBUG_MS5611
#define MS5611_I2C_ADDRESS         0x76

#define MS561101BA_PROM_BASE_ADDR  0xA0
#define MS561101BA_PROM_REG_COUNT  8     // number of registers in the PROM
#define MS561101BA_D1_Pressure     0x40
#define MS561101BA_D2_Temperature  0x50
#define MS561101BA_RESET           0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE       3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256         0x00
#define MS561101BA_OSR_512         0x02
#define MS561101BA_OSR_1024        0x04
#define MS561101BA_OSR_2048        0x06
#define MS561101BA_OSR_4096        0x08

unsigned short MS5611Prom[MS561101BA_PROM_REG_COUNT];

long MS5611lastRawTemperature;
long MS5611lastRawPressure;
int64_t MS5611_sens=0;
int64_t MS5611_offset=0;

// taken from AN520
unsigned char MS5611crc4(unsigned short n_prom[])
{
	unsigned short n_rem = 0;               // crc reminder
	unsigned short crc_read;            // original value of the crc

	crc_read  = n_prom[7];               //save read CRC
	n_prom[7] = (0xFF00 & (n_prom[7])); //CRC byte is replaced by 0

	for (int cnt = 0; cnt < 16; cnt++) {   // operation is performed on bytes
	    // choose LSB or MSB
		if (cnt%2 == 1) {
			n_rem ^= (n_prom[cnt>>1]) & 0x00FF;
		} else {
			n_rem ^= n_prom[cnt>>1] >> 8;
		}

		for (int n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & (0x8000)) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	n_rem = (n_rem >> 12) & 0xF; // // final 4-bit reminder is CRC code

	n_prom[7] = crc_read; // restore the crc_read to its original place

	return (n_rem);
}

int MS5611readPROM(int addr)
{
	for (int i=0;i<MS561101BA_PROM_REG_COUNT; i++) {
		sendByteI2C(addr, MS561101BA_PROM_BASE_ADDR + 2*i);
		if(Wire.requestFrom(addr, 2) == 2) {
			MS5611Prom[i] = readWordI2C();
			//print("%d  %5d\r\n", i, MS5611Prom[i]);
		} else {
			return 0;
		}
	}

	int crc     = MS5611crc4(MS5611Prom);
	int crcProm = MS5611Prom[7] & 0xf;
	//print("crc calculated %d,  prom %d, crc is %s\r\n", crc, crcProm, (crc == crcProm) ? "good" : "bad");
	if(crc == crcProm) {
		return 1;
	}
	return 0;
}

void MS5611reset(int addr) {
	sendByteI2C(addr, MS561101BA_RESET);
}



float pressure			 = 0;
long rawPressure         = 0;
long rawTemperature      = 0;
byte pressureCount       = 0;
float pressureFactor     = 1/5.255;
boolean isReadPressure   = false;
float rawPressureSum     = 0;
byte rawPressureSumCount = 0;

unsigned long MS5611readConversion(int addr) {
  unsigned long conversion = 0;

  // start read sequence
  sendByteI2C(addr, 0);
  Wire.requestFrom(addr, MS561101BA_D1D2_SIZE);
  if(Wire.available() == MS561101BA_D1D2_SIZE) {
    conversion = (readByteI2C() << 16) | (readByteI2C() << 8) | (readByteI2C() << 0);
  } else {
    conversion = 0;
  }

  return conversion;
}


void requestRawTemperature()
{
  sendByteI2C(MS5611_I2C_ADDRESS, MS561101BA_D2_Temperature + MS561101BA_OSR_4096);
}


unsigned long readRawTemperature()
{
  // see datasheet page 7 for formulas
  MS5611lastRawTemperature = MS5611readConversion(MS5611_I2C_ADDRESS);
  int64_t dT       = MS5611lastRawTemperature - (((long)MS5611Prom[5]) << 8);
  MS5611_offset  = (((int64_t)MS5611Prom[2]) << 16) + ((MS5611Prom[4] * dT) >> 7);
  MS5611_sens    = (((int64_t)MS5611Prom[1]) << 15) + ((MS5611Prom[3] * dT) >> 8);


#ifdef DEBUG_MS5611
  Serial.print(" rT: ");
  Serial.print(MS5611lastRawTemperature);
#endif

  return MS5611lastRawTemperature;
}


float readTemperature()
{
  return ((1<<5)*2000 + (((MS5611lastRawTemperature - ((int64_t)MS5611Prom[5] << 8)) * MS5611Prom[6]) >> (23-5))) / ((1<<5) * 100.0);
}

void requestRawPressure()
{
  sendByteI2C(MS5611_I2C_ADDRESS, MS561101BA_D1_Pressure + MS561101BA_OSR_4096);
}

float readRawPressure()
{
  MS5611lastRawPressure = MS5611readConversion(MS5611_I2C_ADDRESS);
#ifdef DEBUG_MS5611
  //print(" dT: %6d  off: %6u p %6u  sens: %6d  ", (int32)dT, (unsigned int32)off, (unsigned int32)(( MS5611lastRawPressure * sens) >> 21), (int32)sens);
  Serial.print(" rP: ");
  Serial.print(MS5611lastRawPressure);
#endif

  return (((( MS5611lastRawPressure * MS5611_sens) >> 21) - MS5611_offset) >> (15-5)) / ((float)(1<<5));
}


void initializeBaro() {
  pressure = 0;
  baroGroundAltitude = 0;
  pressureFactor = 1/5.255;

  MS5611reset(MS5611_I2C_ADDRESS); // reset the device to populate its internal PROM registers
  delay(3); // some safety time

  if(MS5611readPROM(MS5611_I2C_ADDRESS) ) {
	  vehicleState |= BARO_DETECTED;
  }


  requestRawTemperature(); // setup up next measure() for temperature
  isReadPressure = false;
  pressureCount = 0;
  delay(10);
  measureBaroSum(); // read temperature
  delay(10);
  measureBaro(); // read pressure
  delay(10);

  measureGroundBaro();

#if 0
  // check if measured ground altitude is valid
  while (abs(baroRawAltitude - baroGroundAltitude) > 10) {
    delay(26);
    measureGroundBaro();
  }
#endif
  baroAltitude = baroGroundAltitude;
}

void measureBaro() {
  measureBaroSum();
  evaluateBaroAltitude();
}

void measureBaroSum() {
  // switch between pressure and temperature measurements
  // each loop, since it is slow to measure pressure
  if (isReadPressure) {
    rawPressureSum += readRawPressure();
    rawPressureSumCount++;
    if (pressureCount == 20) {
      requestRawTemperature();
      pressureCount = 0;
      isReadPressure = false;
    } else {
      requestRawPressure();
	}
    pressureCount++;
  } else { // select must equal TEMPERATURE
    readRawTemperature();
    requestRawPressure();
    isReadPressure = true;
  }
}

bool MS5611_first_read = true;

void evaluateBaroAltitude() {
  if (rawPressureSumCount == 0) { // it may occur at init time that no pressure has been read yet!
    return;
  }

  pressure = rawPressureSum / rawPressureSumCount;

  baroRawAltitude = 44330 * (1 - pow(pressure/101325.0, pressureFactor)); // returns absolute baroAltitude in meters
  // use calculation below in case you need a smaller binary file for CPUs having just 32KB flash ROM
  // baroRawAltitude = (101325.0-pressure)/4096*346;

  if(MS5611_first_read) {
    baroAltitude = baroRawAltitude;
    MS5611_first_read = false;
  } else {
    baroAltitude = filterSmooth(baroRawAltitude, baroAltitude, baroSmoothFactor);
  }

#ifdef DEBUG_MS5611_b
  Serial.print("  p ");
  Serial.print(pressure);
  Serial.print("  bra ");
  Serial.print(baroRawAltitude);
  Serial.print("  ba ");
  Serial.print(baroAltitude);
  Serial.println();
#endif

  rawPressureSum = 0.0;
  rawPressureSumCount = 0;
}

#endif
