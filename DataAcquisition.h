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

// This header file defines function calls and ISR's needed to communicatw
// over SPI, I2C and other bus communication protocols for measuring sensor data

// ********************************************
// I2C Communication with Wii Sensors
// Original code written by lamarche_mathieu
// Modifications by jihlein 
// ********************************************
// I2C function calls defined in I2C.h
#ifndef AeroQuad_v18
short NWMP_acc[3];
short NWMP_gyro[3];

void Init_Gyro_acc();
void updateControls();

void Init_Gyro_Acc(void) {
  //Init WM+ and Nunchuk
  updateRegisterI2C(0x53, 0xFE, 0x05);
  delay(100);
  updateRegisterI2C(0x53, 0xF0, 0x55);
  delay(100);
};

void updateControls() {
  int i,j;
  unsigned char buffer[6];

  for(j=0;j<2;j++) {
    sendByteI2C(0x52, 0x00);
    Wire.requestFrom(0x52,6);
    for(i = 0; i < 6; i++) 
      buffer[i] = Wire.receive();
    if (buffer[5] & 0x02) { //If WiiMP
      NWMP_gyro[0]= (((buffer[4]>>2)<<8) +  buffer[1])/16;  //hji
      NWMP_gyro[1]= (((buffer[5]>>2)<<8) +  buffer[2])/16;  //hji
      NWMP_gyro[2]=-(((buffer[3]>>2)<<8) +  buffer[0])/16;  //hji
    }
    else {//If Nunchuk
      NWMP_acc[0]=(buffer[2]<<1)|((buffer[5]>>4)&0x01);  //hji
      NWMP_acc[1]=(buffer[3]<<1)|((buffer[5]>>5)&0x01);  //hji
      NWMP_acc[2]=buffer[4];                             //hji
      NWMP_acc[2]=NWMP_acc[2]<<1;                        //hji
      NWMP_acc[2]=NWMP_acc[2] & 0xFFFC;                  //hji
      NWMP_acc[2]=NWMP_acc[2]|((buffer[5]>>6)&0x03);     //hji
    }
  }
}
#endif

#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    #include <CHR6DM.h>
    CHR6DM chr6dm;

    void initCHR6DM(){
        Serial1.begin(115200); //is this needed here? it's already done in Setup, APM TX1 is closest to board edge, RX1 is one step in (green TX wire from CHR goes into APM RX1)
        chr6dm.resetToFactory();
        chr6dm.setListenMode();
        chr6dm.setActiveChannels(CHANNEL_ALL_MASK);
        chr6dm.requestPacket();
    }

    void readCHR6DM(){
        chr6dm.waitForAndReadPacket();
        chr6dm.requestPacket();
    }
#endif



