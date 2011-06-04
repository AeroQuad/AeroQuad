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

#ifndef _AEROQUAD_PLATFORM_WII_H_
#define _AEROQUAD_PLATFORM_WII_H_

#include <WProgram.h>

class Platform_Wii
{
private:
  short accel[3];
  short gyro[3];
  byte  wmpSlow[3];


public:

  void initialize(boolean paris3Board = false) 
  {
	if (paris3Board) {
	  pinMode(12, OUTPUT);
      digitalWrite(12, LOW);
      delay(200);
      digitalWrite(12, HIGH);
      delay(100);
	}
	
    //Init WM+ and Nunchuk
    updateRegisterI2C(0x53, 0xF0, 0x55);
    delay(100);
    updateRegisterI2C(0x53, 0xFE, 0x05);
    delay(100);
  };

  void measure() 
  {
    unsigned char buffer[6];

    for(byte j=0;j<2;j++) {
      sendByteI2C(0x52, 0x00);
      Wire.requestFrom(0x52,6);
	  
      for(byte i = 0; i < 6; i++) 
        buffer[i] = Wire.receive();
		
      if ((buffer[5] & 0x02) == 0x02 && (buffer[5]&0x01) == 0) { //If WiiMP
        gyro[ROLL]  = (((buffer[5]>>2)<<8) +  buffer[2]);  // Configured for Paris MultiWii Board
        gyro[PITCH] = (((buffer[4]>>2)<<8) +  buffer[1]);  // Configured for Paris MultiWii Board
        gyro[YAW]   = (((buffer[3]>>2)<<8) +  buffer[0]);  // Configured for Paris MultiWii Board
      
        wmpSlow[ROLL]  = (buffer[4] & 0x02) >> 1 ;
        wmpSlow[PITCH] = (buffer[3] & 0x01) >> 0 ;
        wmpSlow[YAW]   = (buffer[3] & 0x02) >> 1 ;
      }
      else if ((buffer[5]&0x02) == 0 && (buffer[5]&0x01) == 0) {//If Nunchuk
        accel[XAXIS] = (buffer[2]<<1)|((buffer[5]>>4)&0x01);  // Configured for Paris MultiWii Board
        accel[YAXIS] = (buffer[3]<<1)|((buffer[5]>>5)&0x01);  // Configured for Paris MultiWii Board
        accel[ZAXIS] = buffer[4];                           
        accel[ZAXIS] = accel[2]<<1;                    
        accel[ZAXIS] = accel[2] & 0xFFFC;                
        accel[ZAXIS] = accel[2]|((buffer[5]>>6)&0x03);     // Configured for Paris MultiWii Board   
      }
      if (j == 0) 
	    delay(3);
    }
  }

  short getAccelADC(byte axis)
  {
    return accel[axis];
  }

  short getGyroADC(byte axis)
  {
    return gyro[axis];
  }
  
  byte getWmpSlow(byte axis)
  {
    return wmpSlow[axis];
  }

};

#endif