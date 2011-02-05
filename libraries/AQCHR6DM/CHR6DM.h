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

// Written by Lokling & Honk: http://aeroquad.com/showthread.php?1287-Experimental-CHR6DM-sensor-board

// Usage: define a global var such as  "CHR6DM chr6 ;" in Aeroquad.pde
// Values can then be read such as chr6.data.pitch and so on

#ifndef _CHR6DM_H_
#define _CHR6DM_H_

#include <Wire.h>
#include <stdlib.h>
#include <math.h>
#include "WProgram.h"
#include "pins_arduino.h"

#define DEFAULT_TIMEOUT 1000

// Null packet
const int  NO_DATA                   = 0x00;
const int  FAILED_CHECKSUM           = 0x01;

 
// Client command packets
const int  SET_ACTIVE_CHANNELS       =  0x80;
const int  SET_SILENT_MODE           =  0x81;
const int  SET_BROADCAST_MODE        =  0x82;
const int  SET_GYRO_BIAS             =  0x83;
const int  SET_ACCEL_BIAS            =  0x84;
const int  SET_ACCEL_REF_VECTOR      =  0x85;
const int  AUTO_SET_ACCEL_REF        =  0x86;
const int  ZERO_RATE_GYROS           =  0x87;
const int  SELF_TEST                 =  0x88;
const int  SET_START_CAL             =  0x89;
const int  SET_PROCESS_COVARIANCE    =  0x8A;
const int  SET_MAG_COVARIANCE        =  0x8B;
const int  SET_ACCEL_COVARIANCE      =  0x8C;
const int  SET_EKF_CONFIG            =  0x8D;
const int  SET_GYRO_ALIGNMENT        =  0x8E;
const int  SET_ACCEL_ALIGNMENT       =  0x8F;
const int  SET_MAG_REF_VECTOR        =  0x90;
const int  AUTO_SET_MAG_REF          =  0x91;
const int  SET_MAG_CAL               =  0x92;
const int  SET_MAG_BIAS              =  0x93;
const int  SET_GYRO_SCALE            =  0x94;
const int  EKF_RESET                 =  0x95;
const int  RESET_TO_FACTORY          =  0x96;
const int  WRITE_TO_FLASH            =  0xA0;
const int  GET_DATA                  =  0x01;
const int  GET_ACTIVE_CHANNELS       =  0x02;
const int  GET_BROADCAST_MODE        =  0x03;
const int  GET_ACCEL_BIAS            =  0x04;
const int  GET_ACCEL_REF_VECTOR      =  0x05;
const int  GET_GYRO_BIAS             =  0x06;
const int  GET_GYRO_SCALE            =  0x07;
const int  GET_START_CAL             =  0x08;
const int  GET_EKF_CONFIG            =  0x09;
const int  GET_ACCEL_COVARIANCE      =  0x0A;
const int  GET_MAG_COVARIANCE        =  0x0B;
const int  GET_PROCESS_COVARIANCE    =  0x0C;
const int  GET_STATE_COVARIANCE      =  0x0D;
const int  GET_GYRO_ALIGNMENT        =  0x0E;
const int  GET_ACCEL_ALIGNMENT       =  0x0F;
const int  GET_MAG_REF_VECTOR        =  0x10;
const int  GET_MAG_CAL               =  0x11;
const int  GET_MAG_BIAS              =  0x12;

// Board status and data packets
const int  COMMAND_COMPLETE          =  0xB0;
const int  COMMAND_FAILED            =  0xB1;
const int  BAD_CHECKSUM              =  0xB2;
const int  BAD_DATA_LENGTH           =  0xB3;
const int  UNRECOGNIZED_PACKET       =  0xB4;
const int  BUFFER_OVERFLOW           =  0xB5;
const int  STATUS_REPORT             =  0xB6;
const int  SENSOR_DATA               =  0xB7;
const int  GYRO_BIAS_REPORT          =  0xB8;
const int  GYRO_SCALE_REPORT         =  0xB9;
const int  START_CAL_REPORT          =  0xBA;
const int  ACCEL_BIAS_REPORT         =  0xBB;
const int  ACCEL_REF_VECTOR_REPORT   =  0xBC;
const int  ACTIVE_CHANNEL_REPORT     =  0xBD;
const int  ACCEL_COVARIANCE_REPORT   =  0xBE;
const int  MAG_COVARIANCE_REPORT     =  0xBF;
const int  PROCESS_COVARIANCE_REPORT =  0xC0;
const int  STATE_COVARIANCE_REPORT   =  0xC1;
const int  EKF_CONFIG_REPORT         =  0xC2;
const int  GYRO_ALIGNMENT_REPORT     =  0xC3;
const int  ACCEL_ALIGNMENT_REPORT    =  0xC4;
const int  MAG_REF_VECTOR_REPORT     =  0xC5;
const int  MAG_CAL_REPORT            =  0xC6;
const int  MAG_BIAS_REPORT           =  0xC7;
const int  BROADCAST_MODE_REPORT     =  0xC8;


const int  CHANNEL_YAW_MASK           = 1<<15;
const int  CHANNEL_PITCH_MASK         = 1<<14;
const int  CHANNEL_ROLL_MASK          = 1<<13;
const int  CHANNEL_YAW_RATE_MASK      = 1<<12;
const int  CHANNEL_PITCH_RATE_MASK    = 1<<11;
const int  CHANNEL_ROLL_RATE_MASK     = 1<<10;
const int  CHANNEL_MX_MASK            = 1<<9;
const int  CHANNEL_MY_MASK            = 1<<8;
const int  CHANNEL_MZ_MASK            = 1<<7;
const int  CHANNEL_GX_MASK            = 1<<6;
const int  CHANNEL_GY_MASK            = 1<<5;
const int  CHANNEL_GZ_MASK            = 1<<4;
const int  CHANNEL_AY_MASK            = 1<<3;
const int  CHANNEL_AX_MASK            = 1<<2;
const int  CHANNEL_AZ_MASK            = 1<<1;
const int  CHANNEL_ALL_MASK           = 65535;


// Scale factors
const double SCALE_YAW        = 0.0109863; // �/LSB
const double SCALE_PITCH      = 0.0109863;
const double SCALE_ROLL       = 0.0109863;
const double SCALE_YAW_RATE   = 0.0137329; // �/s/LSB
const double SCALE_PITCH_RATE = 0.0137329;
const double SCALE_ROLL_RATE  = 0.0137329;
const double SCALE_MAG_X      = 0.061035; // mGauss/LSB
const double SCALE_MAG_Y      = 0.061035;
const double SCALE_MAG_Z      = 0.061035;
const double SCALE_GYRO_X     = 0.01812; // �/s/LSB
const double SCALE_GYRO_Y     = 0.01812;
const double SCALE_GYRO_Z     = 0.01812;
const double SCALE_ACCEL_X    = 0.106812; // mg/LSB
const double SCALE_ACCEL_Y    = 0.106812;
const double SCALE_ACCEL_Z    = 0.106812;

const char PACKET_HEADER[] = {'s','n','p'};
const int HEADER_CHECKSUM = 's'+'n'+'p';






struct Data
{
	 bool yawEnabled;
	 bool pitchEnabled;
	 bool rollEnabled;
	 bool yawRateEnabled;
	 bool pitchRateEnabled;
	 bool rollRateEnabled;
	 bool mxEnabled;
	 bool myEnabled;
	 bool mzEnabled;
	 bool gxEnabled;
	 bool gyEnabled;
	 bool gzEnabled;
	 bool axEnabled;
	 bool ayEnabled;
	 bool azEnabled;

	 double yaw;
	 double pitch;
	 double roll;
	 double yawRate;
	 double pitchRate;
	 double rollRate;
	 double mx;
	 double my;
	 double mz;
	 double gx;
	 double gy;
	 double gz;
	 double ax;
	 double ay;
	 double az;
};




class CHR6DM 
{
private:
	int packet[100];
	int packet_length;

public:
	Data data;

	CHR6DM(void);    

	void EKFReset();

	void writeToFlash();

	int readPacket();  

	int blockingRead();

	bool syncToHeader();

	void resetToFactory();

	bool setActiveChannels(int channels);

	void setBroadCastMode(int x);

	void sendPacket(int command);

	void sendPacket(int command, int* bytes, int byteslength);

	bool requestPacket();

	bool waitForAndReadPacket();

	bool requestAndReadPacket();

	bool waitFor(int command,int timeout); 

	bool decodePacket();

	bool selfTest();

	int bytesToSignedShort(int high, int low);

	bool setListenMode();

	bool waitForAck(int timeout);
};

#endif