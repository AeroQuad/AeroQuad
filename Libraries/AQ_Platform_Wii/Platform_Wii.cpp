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

#include "Platform_Wii.h"
#include <Device_I2C.h>

void Platform_Wii::initialize() 
{
  //Init WM+ and Nunchuk
  updateRegisterI2C(0x53, 0xFE, 0x05);
  delay(100);
  updateRegisterI2C(0x53, 0xF0, 0x55);
  delay(100);
};

void Platform_Wii::measure() 
{
  unsigned char buffer[6];

  for(byte j=0;j<2;j++) {
    sendByteI2C(0x52, 0x00);
    Wire.requestFrom(0x52,6);
    for(byte i = 0; i < 6; i++) 
      buffer[i] = Wire.receive();
    if (buffer[5] & 0x02) { //If WiiMP
      gyro[0]= (((buffer[4]>>2)<<8) +  buffer[1])/16;  //hji
      gyro[1]= (((buffer[5]>>2)<<8) +  buffer[2])/16;  //hji
      gyro[2]=-(((buffer[3]>>2)<<8) +  buffer[0])/16;  //hji
    }
    else {//If Nunchuk
      accel[0]=(buffer[2]<<1)|((buffer[5]>>4)&0x01);  //hji
      accel[1]=(buffer[3]<<1)|((buffer[5]>>5)&0x01);  //hji
      accel[2]=buffer[4];                             //hji
      accel[2]=accel[2]<<1;                        //hji
      accel[2]=accel[2] & 0xFFFC;                  //hji
      accel[2]=accel[2]|((buffer[5]>>6)&0x03);     //hji
    }
  }
}

short Platform_Wii::getAccelADC(byte axis)
{
  return accel[axis];
}

short Platform_Wii::getGyroADC(byte axis)
{
  return gyro[axis];
}
