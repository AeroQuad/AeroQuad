/*
  AeroQuad v2.4 - April 2011
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

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#ifndef _AEROQUAD_PLATFORM_CHR6DM_H_
#define _AEROQUAD_PLATFORM_CHR6DM_H_

#include "Arduino.h"

#define DEFAULT_TIMEOUT 1000



// Null packet

     const int  NO_DATA                   = 0x00;
     const int  FAILED_CHECKSUM           = 0x01;


     int packet[100];
     int packet_length = 0;

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

     /**
       * Takes the scale factors from the CHR6DM spec sheet
       * and then converts them to the appropriate unit
       * we are using.
       * http://www.chrobotics.com/docs/chr6dm_datasheet.pdf
       */
     const double SCALE_YAW         = 0.0109863 * DEG_TO_RAD;       // All angles are in radians
     const double SCALE_PITCH       = 0.0109863 * DEG_TO_RAD;
     const double SCALE_ROLL        = 0.0109863 * DEG_TO_RAD;
     const double SCALE_YAW_RATE    = 0.0137329 * DEG_TO_RAD;       // All angle rates are in radians
     const double SCALE_PITCH_RATE  = 0.0137329 * DEG_TO_RAD;
     const double SCALE_ROLL_RATE   = 0.0137329 * DEG_TO_RAD;
     const double SCALE_MAG_X       = 0.061035;                     // mGauss/LSB
     const double SCALE_MAG_Y       = 0.061035;
     const double SCALE_MAG_Z       = 0.061035;
     const double SCALE_GYRO_X      = 0.01812;                      // degrees/s/LSB
     const double SCALE_GYRO_Y      = 0.01812;
     const double SCALE_GYRO_Z      = 0.01812;
     const double SCALE_ACCEL_X     = (0.106812 * 9.8065) / 1000;   // converts millie G-Forces to meters/s^2
     const double SCALE_ACCEL_Y     = (0.106812 * 9.8065) / 1000;
     const double SCALE_ACCEL_Z     = (0.106812 * 9.8065) / 1000; 
    
     const char PACKET_HEADER[] = {'s','n','p'};
     const int HEADER_CHECKSUM = 's'+'n'+'p';


#include <Wire.h>




class Data{

public:
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




class CHR6DM {

public:


    CHR6DM(void){
        //Nothing here
    }



    Data data;

  void EKFReset() {
    sendPacket(EKF_RESET);
    waitForAck(DEFAULT_TIMEOUT);
  }

  void writeToFlash() {
        sendPacket(WRITE_TO_FLASH);
        waitForAck(5000);
  }


    int readPacket()  {

        if (!syncToHeader()){
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

        for (int i = 1; i <= dataBytes ;i++ ){
            packet[i] = blockingRead() ;
            calculatedChecksum+=packet[i];
        }

        int high =  blockingRead();
        int low =   blockingRead();

        int packetChecksum = bytesToSignedShort(high,low);

        if (calculatedChecksum!=packetChecksum) {
            //Serial.print("Bad checksum ");Serial.print(" calculated="); Serial.print(calculatedChecksum);Serial.print(" actual="); Serial.println(packetChecksum);
            packet[0] = FAILED_CHECKSUM;
            packet_length=1;
            return FAILED_CHECKSUM;
        }

        packet_length=length;
        return packet[0];

    }
    
    int blockingRead(){
        int read=-1;

         long starttime = millis();
         while(read==-1 && (millis()-starttime)<100){
            read = Serial1.read();
         }

         return read;
    }

    bool syncToHeader()  {

        int MAX_PACKET_LENGTH = 41; // TODO - Unsure about this, calculate the actual
        int available = Serial1.available();

        if (available > MAX_PACKET_LENGTH ){
            for (int i = 0; i < available - MAX_PACKET_LENGTH; i++){
                Serial1.read();
            }
        }

        while (Serial1.available()>0){
            if (blockingRead()==PACKET_HEADER[0] && blockingRead()==PACKET_HEADER[1] && blockingRead()==PACKET_HEADER[2] ) return true;
        }

        return false;
    }


     void resetToFactory()  {
        sendPacket(RESET_TO_FACTORY);
    }

     bool setActiveChannels(int channels)  {
        sendPacket(SET_ACTIVE_CHANNELS,(int[]){channels},1);
        return waitForAck(DEFAULT_TIMEOUT);
    }


     void setBroadCastMode(int x) {
        sendPacket(SET_BROADCAST_MODE,(int[]){x},1);
    }

     void sendPacket(int command)  {
        sendPacket(command,0,0);
    }

     void sendPacket(int command, int* bytes, int byteslength)  {

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

    bool requestPacket(){
        sendPacket(GET_DATA);
    }

    bool waitForAndReadPacket(){
        waitFor(SENSOR_DATA, DEFAULT_TIMEOUT);
    }

     bool requestAndReadPacket() {
        requestPacket();
        return waitForAndReadPacket();
     }


     bool waitFor(int command,int timeout) {

       long startTime = millis();
        while((millis()-startTime)<timeout){
            int packetType  = readPacket();

            if (packetType>1){
                bool result = decodePacket();

                if (packetType==command){
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

     bool decodePacket() {
        int index = 0;
        switch (packet[index++]) {
            case SENSOR_DATA: {

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

                if (index!=packet_length){
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


     bool selfTest(){
        sendPacket(SELF_TEST);
        return waitFor(STATUS_REPORT,DEFAULT_TIMEOUT);
    }

    int bytesToSignedShort(int high, int low) {
        return word(high,low);
    }

    bool setListenMode() {
        sendPacket(SET_SILENT_MODE);
        return waitForAck(DEFAULT_TIMEOUT);
    }

    bool waitForAck(int timeout) {

        long startTime = millis();
        while(millis()-startTime<timeout){
        readPacket();
        int command=packet[0];
            switch(command){
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
	
	void read(){
        waitForAndReadPacket();
        requestPacket();
    }

};


#endif  // #ifndef _AEROQUAD_PLATFORM_CHR6DM_H_

#endif  // #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
