////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readWii(int readWithZeroBias) {  // readWithZeroBias = 0 when performing sensor calibration reads
  int i,j;                            // readWithZeroBias = 1 when performing sensor normal reads
  unsigned char buffer[6];

  for(j=0;j<2;j++) {
    sendByteI2C(0x52, 0x00);
    Wire.requestFrom(0x52,6);
    for(i = 0; i < 6; i++) 
      buffer[i] = Wire.receive();
    if (buffer[5] & 0x02) {
      // The following 3 lines read the gyro and assign it's data to gyroRaw
      // in the correct order and phase to suit the standard WMP installation
      // orientation.  See TBD for details.  If your WMP is not installed in this
      // orientation, this is where you make the changes.
      rateGyro.gyroRaw[ROLL]  = (-rateGyro.gyroZero[ROLL] * readWithZeroBias) - ((((buffer[5]>>2)<<8) +  buffer[2])/16);
      rateGyro.gyroRaw[PITCH] = ((((buffer[4]>>2)<<8) +  buffer[1])/16)       - (rateGyro.gyroZero[PITCH] * readWithZeroBias);
      rateGyro.gyroRaw[YAW]   = (-rateGyro.gyroZero[YAW] * readWithZeroBias)  - ((((buffer[3]>>2)<<8) +  buffer[0])/16);
      delayMicroseconds(750);
    }
    else {
      // The following lines  read the accelerometer and assign it's data to accelRaw
      // in the correct order and phase to suit the standard Wii Nunchuk installation
      // orientation.  See TBD for details.  If your Wii Nunchuk is not installed in this
      // orientation, this is where you make the changes.
      accel.accelRaw[XAXIS] = (-accel.accelZero[XAXIS] * readWithZeroBias) - ((buffer[3]<<1)|((buffer[5]>>5)&0x01));         // **
      accel.accelRaw[YAXIS] = (-accel.accelZero[YAXIS] * readWithZeroBias) - ((buffer[2]<<1)|((buffer[5]>>4)&0x01));         // **
      accel.accelRaw[ZAXIS] = buffer[4];
      accel.accelRaw[ZAXIS] = accel.accelRaw[ZAXIS]<<1;
      accel.accelRaw[ZAXIS] = accel.accelRaw[ZAXIS] & 0xFFFC;
      accel.accelRaw[ZAXIS] = (-accel.accelZero[ZAXIS] * readWithZeroBias) - (accel.accelRaw[ZAXIS]|((buffer[5]>>6)&0x03));  // **
      delayMicroseconds(750);
    }
  }
  for (byte axis = 0; axis < LASTAXIS; axis++) {
    rateGyro.gyroVector[axis] = smooth(rateGyro.gyroRaw[axis] * rateGyro.gyroScaleFactor, rateGyro.gyroVector[axis], rateGyro.smoothFactor);
    accel.accelVector[axis]   = smooth(  accel.accelRaw[axis] *   accel.accelScaleFactor,   accel.accelVector[axis],    accel.smoothFactor);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
