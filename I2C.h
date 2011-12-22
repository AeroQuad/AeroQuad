/*
  AeroQuad v2.5.1 - December 2011
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

// I2C functions

void sendByteI2C(int deviceAddress, byte dataValue) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
}

byte readByteI2C(int deviceAddress) {
    Wire.requestFrom(deviceAddress, 1);
    return Wire.read();
}

int readWordI2C(int deviceAddress) {
  Wire.requestFrom(deviceAddress, 2);
  return (Wire.read() << 8) | Wire.read();
}

int readWordWaitI2C(int deviceAddress) {
  unsigned char msb, lsb;
  Wire.requestFrom(deviceAddress, 2); // request two bytes
  while(!Wire.available()); // wait until data available
  msb = Wire.read();
  while(!Wire.available()); // wait until data available
  lsb = Wire.read();
  return (((int)msb<<8) | ((int)lsb));
}

int readReverseWordI2C(int deviceAddress) {
  byte lowerByte;
  Wire.requestFrom(deviceAddress, 2);
  lowerByte = Wire.read();
  return (Wire.read() << 8) | lowerByte;
}

byte readWhoI2C(int deviceAddress) {
  // read the ID of the I2C device
  Wire.beginTransmission(deviceAddress);
  Wire.write((byte)0x00);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(deviceAddress, 1);
  return Wire.read();
}

void updateRegisterI2C(int deviceAddress, byte dataAddress, byte dataValue) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(dataAddress);
  Wire.write(dataValue);
  Wire.endTransmission();
}  


