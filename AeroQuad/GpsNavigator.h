/*
  AeroQuad v3.0 - Febuary 2012
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)


#ifndef _AQ_Navigator_H_
#define _AQ_Navigator_H_

boolean isHomeBaseInitialized() {
  return homePosition.latitude != GPS_INVALID_ANGLE;
}

void initHomeBase() {
  
  homePosition.latitude = currentPosition.latitude;
  homePosition.longitude = currentPosition.longitude;
}

boolean haveMission() {
  return missionNbPoint != 0;
}

float gpsSpeedSmoothValue = 0.5;
float gpsCoarseSmoothValue = 0.5;

#define MAX_NAVIGATON_SPEED 400  // m/s * 100 // 3 m/s = 10.8km/h

void processPositionCorrection() {
  
  float derivateDistanceX = (currentPosition.longitude - previousPosition.longitude)*0.649876;
  float derivateDistanceY = (currentPosition.latitude - previousPosition.latitude)*1.113195;
  float derivateDistance = sqrt(sq(derivateDistanceY) + sq(derivateDistanceX));
  
  float distanceX = (currentPosition.longitude - positionToReach.longitude)*0.649876;
  float distanceY = (currentPosition.latitude - positionToReach.latitude)*1.113195;
  float distance = sqrt(sq(derivateDistanceY) + sq(derivateDistanceX));
  
  gpsLaggedSpeed = gpsLaggedSpeed * (gpsSpeedSmoothValue) + derivateDistance * (1-gpsSpeedSmoothValue);
  if (derivateDistanceX != 0 || derivateDistanceY != 0) {
    float tmp = degrees(atan2(derivateDistanceX, derivateDistanceY));
      if (tmp < 0) {
        tmp += radians(360);
      }
      gpsLaggedCourse = (int)((float)gpsLaggedCourse*(gpsCoarseSmoothValue) + tmp*100*(1-gpsCoarseSmoothValue));
  }
  
  float angleToWaypoint = atan2(distanceX, distanceY);
  float courseRads = radians(gpsLaggedCourse/100);
  
  float azimuth = getAbsoluteHeading();
  float currentSpeedCmPerSecRoll = sin(courseRads-azimuth)*gpsLaggedSpeed; 
  float currentSpeedCmPerSecPitch = cos(courseRads-azimuth)*gpsLaggedSpeed;
  
  if (distance != 0) {
    
    float angle = angleToWaypoint-azimuth;
    float tmpsin = sin(angle);
    float tmpcos = cos(angle);
    
    float maxSpeedRoll = 0.0;
    float maxSpeedPitch = 0.0;
    if (distance > 300) //if distance is over 20m, use max speed
    {
      maxSpeedRoll = MAX_NAVIGATON_SPEED*tmpsin; //max speed on roll
      maxSpeedPitch = MAX_NAVIGATON_SPEED*tmpcos; //max speed on pitch
      
      gpsRollAxisCorrection = updatePID(maxSpeedRoll, currentSpeedCmPerSecRoll, &PID[GPSROLL_PID_IDX]);
      gpsPitchAxisCorrection = updatePID(maxSpeedPitch, currentSpeedCmPerSecPitch , &PID[GPSPITCH_PID_IDX]);
    }
    
//    else {
//      maxSpeedRoll = (MAX_NAVIGATON_SPEED*tmpsin*((float)distance/300)); //roll
//      maxSpeedPitch = (MAX_NAVIGATON_SPEED*tmpcos*((float)distance/300)); //pitch
//    }
  }

  
  
  previousPosition.latitude = currentPosition.latitude;
  previousPosition.longitude = currentPosition.longitude;
}



#endif

