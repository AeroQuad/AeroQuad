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
  _gyro->measure(); // defined in Gyro.h
  _accel->measure(); // defined in Accel.h
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    _compass->measure(_flightAngle->getData(ROLL),_flightAngle->getData(PITCH));
  #endif
 
  // ********************* Read Slower Sensors *******************
  #if defined(HeadingMagHold)
    if (_currentTime > _compassTime) 
    {
      _compass->measure(_flightAngle->getData(ROLL),_flightAngle->getData(PITCH)); // defined in compass.h
      _compassTime = _currentTime + COMPASSLOOPTIME;
    }
  #endif
  #if defined(AltitudeHold)
    if (_currentTime > _altitudeTime) 
    {
      _altitudeProvider->measure(); // defined in altitude.h
      _altitudeTime = _currentTime + ALTITUDELOOPTIME;
    }
  #endif
  #if defined(BattMonitor)
    if (_currentTime > _batteryTime) 
    {
      _batteryMonitor->measure(_armed,_throttle);
      _batteryTime = _currentTime + BATTERYLOOPTIME;
    }
  #endif
  
  // ****************** Calculate Absolute Angle *****************
  _flightAngle->calculate(G_Dt); // defined in FlightAngle.h
}

void initSensorsFromEEPROM()
{
  _accel->setSmoothFactor(readFloat(ACCSMOOTH_ADR));
  _accel->setOneG(readFloat(ACCEL1G_ADR));
  _accel->setZero(ROLL, readFloat(LEVELROLLCAL_ADR));
  _accel->setZero(PITCH, readFloat(LEVELPITCHCAL_ADR));
  _accel->setZero(ZAXIS, readFloat(LEVELZCAL_ADR));
  
  _gyro->setSmoothFactor(readFloat(GYROSMOOTH_ADR));
  _gyro->setZero(ROLL,  readFloat(GYRO_ROLL_ZERO_ADR));
  _gyro->setZero(PITCH, readFloat(GYRO_PITCH_ZERO_ADR));
  _gyro->setZero(ZAXIS, readFloat(GYRO_YAW_ZERO_ADR));
}

void storeSensorsToEEPROM()
{
  writeFloat(_accel->getSmoothFactor(),ACCSMOOTH_ADR);
  writeFloat(_accel->getOneG(), ACCEL1G_ADR);
  writeFloat(_accel->getZero(ROLL),  LEVELROLLCAL_ADR);
  writeFloat(_accel->getZero(PITCH), LEVELPITCHCAL_ADR);
  writeFloat(_accel->getZero(ZAXIS), LEVELZCAL_ADR);
  
  writeFloat(_gyro->getSmoothFactor(),GYROSMOOTH_ADR);
  writeFloat(_gyro->getZero(ROLL),  GYRO_ROLL_ZERO_ADR);
  writeFloat(_gyro->getZero(PITCH), GYRO_PITCH_ZERO_ADR);
  writeFloat(_gyro->getZero(ZAXIS), GYRO_YAW_ZERO_ADR);
}

void initTransmitterFromEEPROM()
{
  _receiver->setXmitFactor(readFloat(XMITFACTOR_ADR));
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
  {
    byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
    _receiver->setTransmitterSlope(channel,readFloat(offset+0));     // _mTransmitter[channel] = readFloat(offset+0);
    _receiver->setTransmitterOffset(channel,readFloat(offset+4));    // _bTransmitter[channel] = readFloat(offset+4);
    _receiver->setSmoothFactor(channel,readFloat(offset+8));         //_transmitterSmooth[channel] = readFloat(offset+8);
  }
}

void storeTransmitterToEEPROM()
{
  writeFloat(_receiver->getXmitFactor(),XMITFACTOR_ADR);
  for(byte channel = ROLL; channel < LASTCHANNEL; channel++) 
  {
    byte offset = 12*channel + NVM_TRANSMITTER_SCALE_OFFSET_SMOOTH;
    writeFloat(_receiver->getTransmitterSlope(channel),offset+0);     // _mTransmitter[channel] = readFloat(offset+0);
    writeFloat(_receiver->getTransmitterOffset(channel),offset+4);    // _bTransmitter[channel] = readFloat(offset+4);
    writeFloat(_receiver->getSmoothFactor(channel),offset+8);         //_transmitterSmooth[channel] = readFloat(offset+8);
  }
}



