/*
  AeroQuad v2.2 - Feburary 2011
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

// Sensors.pde is responsible for taking on board sensor measuremens of the AeroQuad

void readSensors() 
{
  // *********************** Read Critical Sensors **********************
  // Apply low pass filter to sensor values and center around zero
  gyro->measure(); // defined in Gyro.h
  accel->measure(); // defined in Accel.h
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    compass->measure(flightAngle->getData(ROLL),flightAngle->getData(PITCH));
  #endif
 
  // ********************* Read Slower Sensors *******************
  #if defined(HeadingMagHold)
    if (currentTime > compassTime) 
    {
      compass->measure(flightAngle->getData(ROLL),flightAngle->getData(PITCH)); // defined in compass.h
      compassTime = currentTime + COMPASSLOOPTIME;
    }
  #endif
  #if defined(AltitudeHold)
    if (currentTime > altitudeTime) 
    {
      altitudeProvider->measure(); // defined in altitude.h
      altitudeTime = currentTime + ALTITUDELOOPTIME;
    }
  #endif
  #if defined(BattMonitor)
    if (currentTime > batteryTime) 
    {
      batteryMonitor->measure(armed,throttle);
      batteryTime = currentTime + BATTERYLOOPTIME;
    }
  #endif
  
  // ****************** Calculate Absolute Angle *****************
  flightAngle->calculate(G_Dt); // defined in FlightAngle.h
}

void initSensorsFromEEPROM()
{
  accel->setSmoothFactor(readFloat(ACCSMOOTH_ADR));
  accel->setOneG(readFloat(ACCEL1G_ADR));
  accel->setZero(ROLL, readFloat(LEVELROLLCAL_ADR));
  accel->setZero(PITCH, readFloat(LEVELPITCHCAL_ADR));
  accel->setZero(ZAXIS, readFloat(LEVELZCAL_ADR));
  
  gyro->setSmoothFactor(readFloat(GYROSMOOTH_ADR));
  gyro->setZero(ROLL,  readFloat(GYRO_ROLL_ZERO_ADR));
  gyro->setZero(PITCH, readFloat(GYRO_PITCH_ZERO_ADR));
  gyro->setZero(ZAXIS, readFloat(GYRO_YAW_ZERO_ADR));
}

void storeSensorsToEEPROM()
{
  writeFloat(accel->getSmoothFactor(),ACCSMOOTH_ADR);
  writeFloat(accel->getOneG(), ACCEL1G_ADR);
  writeFloat(accel->getZero(ROLL),  LEVELROLLCAL_ADR);
  writeFloat(accel->getZero(PITCH), LEVELPITCHCAL_ADR);
  writeFloat(accel->getZero(ZAXIS), LEVELZCAL_ADR);
  
  writeFloat(gyro->getSmoothFactor(),GYROSMOOTH_ADR);
  writeFloat(gyro->getZero(ROLL),  GYRO_ROLL_ZERO_ADR);
  writeFloat(gyro->getZero(PITCH), GYRO_PITCH_ZERO_ADR);
  writeFloat(gyro->getZero(ZAXIS), GYRO_YAW_ZERO_ADR);
}

void initTransmitterFromEEPROM()
{
  receiver->setXmitFactor(readFloat(XMITFACTOR_ADR));
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
  {
    byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
    receiver->setTransmitterSlope(channel,readFloat(offset+0));     // _mTransmitter[channel] = readFloat(offset+0);
    receiver->setTransmitterOffset(channel,readFloat(offset+4));    // _bTransmitter[channel] = readFloat(offset+4);
    receiver->setSmoothFactor(channel,readFloat(offset+8));         //_transmitterSmooth[channel] = readFloat(offset+8);
  }
}

void storeTransmitterToEEPROM()
{
  writeFloat(receiver->getXmitFactor(),XMITFACTOR_ADR);
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
  {
    byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
    writeFloat(receiver->getTransmitterSlope(channel),offset+0);     // _mTransmitter[channel] = readFloat(offset+0);
    writeFloat(receiver->getTransmitterOffset(channel),offset+4);    // _bTransmitter[channel] = readFloat(offset+4);
    writeFloat(receiver->getSmoothFactor(channel),offset+8);         //_transmitterSmooth[channel] = readFloat(offset+8);
  }
}



