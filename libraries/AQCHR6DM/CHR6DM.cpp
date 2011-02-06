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

#include <CHR6DM.h>

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)


void CHR6DM::initCHR6DM()
{
	Serial1.begin(115200); //is this needed here? it's already done in Setup, APM TX1 is closest to board edge, RX1 is one step in (green TX wire from CHR goes into APM RX1)
	resetToFactory();
	setListenMode();
	setActiveChannels(CHANNEL_ALL_MASK);
	requestPacket();
}

void CHR6DM::readCHR6DM()
{
	waitForAndReadPacket();
	requestPacket();
}


CHR6DM::CHR6DM(void)
{
	packet_length = 0;
	CHR_RollAngle = 0.0;
    CHR_PitchAngle = 0.0;
}

void CHR6DM::EKFReset() 
{
	sendPacket(EKF_RESET);
	waitForAck(DEFAULT_TIMEOUT);
}

void CHR6DM::writeToFlash() 
{
	sendPacket(WRITE_TO_FLASH);
	waitForAck(5000);
}


int CHR6DM::readPacket()  
{
	if (!syncToHeader())
	{
		//Serial.println("Not synced to header");
		packet[0]= NO_DATA;
		packet_length=1;
		return NO_DATA;
	}

	int packetType =  blockingRead();
	int dataBytes  =  blockingRead();

	int calculatedChecksum = HEADER_CHECKSUM + packetType + dataBytes;

	int  length = dataBytes+1;
	packet[0] = packetType;

	for (int i = 1; i <= dataBytes ;i++ )
	{
		packet[i] = blockingRead() ;
		calculatedChecksum+=packet[i];
	}

	int high =  blockingRead();
	int low =   blockingRead();

	int packetChecksum = bytesToSignedShort(high,low);

	if (calculatedChecksum!=packetChecksum) 
	{
		//Serial.print("Bad checksum ");Serial.print(" calculated="); Serial.print(calculatedChecksum);Serial.print(" actual="); Serial.println(packetChecksum);
		packet[0] = FAILED_CHECKSUM;
		packet_length=1;
		return FAILED_CHECKSUM;
	}

	packet_length=length;
	return packet[0];
}

int CHR6DM::blockingRead()
{
	int read=-1;

	 long starttime = millis();
	 while(read==-1 && (millis()-starttime)<100)
	 {
		read = Serial1.read();
	 }

	 return read;
}

bool CHR6DM::syncToHeader()  
{

	int MAX_PACKET_LENGTH = 41; // TODO - Unsure about this, calculate the actual
	int available = Serial1.available();

	if (available > MAX_PACKET_LENGTH )
	{
		for (int i = 0; i < available - MAX_PACKET_LENGTH; i++)
		{
			Serial1.read();
		}
	}

	while (Serial1.available()>0)
	{
		if (blockingRead()==PACKET_HEADER[0] && blockingRead()==PACKET_HEADER[1] && blockingRead()==PACKET_HEADER[2] ) return true;
	}

	return false;
}


void CHR6DM::resetToFactory()  
{
	sendPacket(RESET_TO_FACTORY);
}

bool CHR6DM::setActiveChannels(int channels)  
{
	sendPacket(SET_ACTIVE_CHANNELS,(int[]){channels},1);
	return waitForAck(DEFAULT_TIMEOUT);
}


void CHR6DM::setBroadCastMode(int x) 
{
	sendPacket(SET_BROADCAST_MODE,(int[]){x},1);
}

void CHR6DM::sendPacket(int command)  
{
	sendPacket(command,0,0);
}

void CHR6DM::sendPacket(int command, int* bytes, int byteslength)  
{

		int checksum = 0;
		int buffer[] = {'s','n','p',command,byteslength};
		int bufferlength=5;
		for (int i = 0; i < bufferlength; i++) {
			Serial1.write(buffer[i]);
			checksum+=buffer[i];
		}

		for (int i = 0; i < byteslength; i++) {
			Serial1.write(bytes[i]);
			checksum+=bytes[i];

		}

		Serial1.write(checksum>>8);
		Serial1.write(checksum);

}

bool CHR6DM::requestPacket()
{
	sendPacket(GET_DATA);
}

bool CHR6DM::waitForAndReadPacket()
{
	waitFor(SENSOR_DATA, DEFAULT_TIMEOUT);
}

 bool CHR6DM::requestAndReadPacket() 
 {
	requestPacket();
	return waitForAndReadPacket();
 }


 bool CHR6DM::waitFor(int command,int timeout) 
 {

   long startTime = millis();
	while((millis()-startTime)<timeout)
	{
		int packetType  = readPacket();	
		if (packetType>1)
		{
			bool result = decodePacket();

			if (packetType==command)
			{
				return result;
			} /*else {
				Serial.println("Didnt get the expected.. looping");
			}
			*/
		}

	}

	//Serial.println("Timed out !");
	return false;
}

 bool CHR6DM::decodePacket()
 {
	int index = 0;
	switch (packet[index++]) 
	{
		case SENSOR_DATA: 
		{

			int flags = bytesToSignedShort(packet[index++],packet[index++]);

			data.yawEnabled          = (flags & CHANNEL_YAW_MASK            ) == CHANNEL_YAW_MASK;
			data.pitchEnabled        = (flags & CHANNEL_PITCH_MASK          ) == CHANNEL_PITCH_MASK;
			data.rollEnabled         = (flags & CHANNEL_ROLL_MASK           ) == CHANNEL_ROLL_MASK;
			data.yawRateEnabled      = (flags & CHANNEL_YAW_RATE_MASK       ) == CHANNEL_YAW_RATE_MASK;
			data.pitchRateEnabled    = (flags & CHANNEL_PITCH_RATE_MASK     ) == CHANNEL_PITCH_RATE_MASK;
			data.rollRateEnabled     = (flags & CHANNEL_ROLL_RATE_MASK      ) == CHANNEL_ROLL_RATE_MASK;
			data.mxEnabled           = (flags & CHANNEL_MX_MASK             ) == CHANNEL_MX_MASK;
			data.myEnabled           = (flags & CHANNEL_MY_MASK             ) == CHANNEL_MY_MASK;
			data.mzEnabled           = (flags & CHANNEL_MZ_MASK             ) == CHANNEL_MZ_MASK;
			data.gxEnabled           = (flags & CHANNEL_GX_MASK             ) == CHANNEL_GX_MASK;
			data.gyEnabled           = (flags & CHANNEL_GY_MASK             ) == CHANNEL_GY_MASK;
			data.gzEnabled           = (flags & CHANNEL_GZ_MASK             ) == CHANNEL_GZ_MASK;
			data.axEnabled           = (flags & CHANNEL_AX_MASK             ) == CHANNEL_AX_MASK;
			data.ayEnabled           = (flags & CHANNEL_AY_MASK             ) == CHANNEL_AY_MASK;
			data.azEnabled           = (flags & CHANNEL_AZ_MASK             ) == CHANNEL_AZ_MASK;


			if (data.yawEnabled          ){ data.yaw          = SCALE_YAW           * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.pitchEnabled        ){ data.pitch        = SCALE_PITCH         * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.rollEnabled         ){ data.roll         = SCALE_ROLL          * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.yawRateEnabled      ){ data.yawRate      = SCALE_YAW_RATE      * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.pitchRateEnabled    ){ data.pitchRate    = SCALE_PITCH_RATE    * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.rollRateEnabled     ){ data.rollRate     = SCALE_ROLL_RATE     * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.mxEnabled           ){ data.mx           = SCALE_MAG_X         * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.myEnabled           ){ data.my           = SCALE_MAG_Y         * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.mzEnabled           ){ data.mz           = SCALE_MAG_Z         * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.gxEnabled           ){ data.gx           = SCALE_GYRO_X        * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.gyEnabled           ){ data.gy           = SCALE_GYRO_Y        * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.gzEnabled           ){ data.gz           = SCALE_GYRO_Z        * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.axEnabled           ){ data.ax           = SCALE_ACCEL_X       * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.ayEnabled           ){ data.ay           = SCALE_ACCEL_Y       * bytesToSignedShort(packet[index++],packet[index++]); }
			if (data.azEnabled           ){ data.az           = SCALE_ACCEL_Z       * bytesToSignedShort(packet[index++],packet[index++]); }

			if (index!=packet_length)
			{
				//Serial.println("Recevied bad length packet!");
				return false;
			}


			return true;
			}
		case STATUS_REPORT:
			 Serial.println("Received status report");
			 return true;
		case BAD_CHECKSUM:
			 Serial.println("CHR6DM reported bad checksum!");
			 return true;
		case NO_DATA:
			 //Serial.println("CHR6DM No data!");
			 return false;
		case FAILED_CHECKSUM:
			 //Serial.println("CHR6DM reported failed checksum!");
			 return false;
		case COMMAND_COMPLETE:
			Serial.println("COMMAND_COMPLETE");
			return true;
		case COMMAND_FAILED:
			Serial.println("COMMAND_FAILED");
			return false;
		default:
			Serial.print("Received unknown packet ");
			Serial.println(packet[0]);
			return false;

	}
}


bool CHR6DM::selfTest()
{
	sendPacket(SELF_TEST);
	return waitFor(STATUS_REPORT,DEFAULT_TIMEOUT);
}

int CHR6DM::bytesToSignedShort(int high, int low) 
{
	return word(high,low);
}

bool CHR6DM::setListenMode() 
{
	sendPacket(SET_SILENT_MODE);
	return waitForAck(DEFAULT_TIMEOUT);
}

bool CHR6DM::waitForAck(int timeout) 
{

	long startTime = millis();
	while(millis() - startTime<timeout)
	{
		readPacket();
		int command=packet[0];
		switch(command)
		{
			case COMMAND_COMPLETE :
				return true;
			case COMMAND_FAILED:
				break;
			case NO_DATA:
				break;
			default:
			  //Serial.print("Unexcepted packet, waiting for ack:"); Serial.println(command);
			  break;
		}
	}

	Serial.println("Timed out! 2");
	return false;
}

#endif // defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
