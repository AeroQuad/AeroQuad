/*
  AeroQuad v2.3 - February 2011
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

void readSensors(void) {
  // *********************** Read Critical Sensors **********************
  // Apply low pass filter to sensor values and center around zero
  gyro.measure(); // defined in RateGyro.h
  accel.measure(); // defined in Accel.h
  #if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
    compass.measure();
  #endif
 
  // ********************* Read Slower Sensors *******************
  #if defined(COMPASS_INSTALLED)
    if (currentTime > compassTime) {
      compass.measure(kinematics.getAttitude(ROLL), kinematics.getAttitude(PITCH)); // defined in compass.h
      compassTime = currentTime + COMPASSLOOPTIME;
    }
  #endif
  #if defined(AltitudeHold)
    if (currentTime > altitudeTime) {
      altitude.measure(); // defined in altitude.h
      altitudeTime = currentTime + ALTITUDELOOPTIME;
    }
  #endif
  #if defined(BattMonitor)
    if (currentTime > batteryTime) {
      batteryMonitor.measure(armed);
      batteryTime = currentTime + BATTERYLOOPTIME;
    }
  #endif
  
  #if defined(COMPASS_INSTALLED)
    kinematics.calculate(gyro.getNonDriftCorrectedRate(ROLL),  \
                         gyro.getNonDriftCorrectedRate(PITCH), \
                         gyro.getNonDriftCorrectedRate(YAW),   \
                         accel.getData(XAXIS),                     \
                         accel.getData(YAXIS),                     \
                         accel.getData(ZAXIS),                     \
                         accel.getOneG(),                          \
                         compass.getHdgXY(XAXIS),                  \
                         compass.getHdgXY(YAXIS));
  #else
    kinematics.calculate(gyro.getNonDriftCorrectedRate(ROLL),  \
                         gyro.getNonDriftCorrectedRate(PITCH), \
                         gyro.getNonDriftCorrectedRate(YAW),   \
                         accel.getData(XAXIS),                     \
                         accel.getData(YAXIS),                     \
                         accel.getData(ZAXIS),                     \
                         accel.getOneG(),                          \
                         0.0,                                      \
                         0.0);
  
  #endif
}


