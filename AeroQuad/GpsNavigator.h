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

// @todo, Kenny, remove this and put it in the EEPROM
#define MAX_WAYPOINTS 16
GeodeticPosition waypoint[MAX_WAYPOINTS];

int currentWaypoint;

boolean isHomeBaseInitialized() {
  return homePosition.latitude != GPS_INVALID_ANGLE;
}


byte countToInitHome = 0;
#define MIN_NB_GPS_READ_TO_INIT_HOME 15

void initHomeBase() {
  if (isGpsHaveANewPosition) {
    if (countToInitHome < MIN_NB_GPS_READ_TO_INIT_HOME) {
      countToInitHome++;
    }
    else {
      homePosition.latitude = currentPosition.latitude;
      homePosition.longitude = currentPosition.longitude;
      homePosition.altitude = 0;
    }  
  }
}

boolean haveMission() {
  return missionNbPoint != 0;
}

float gpsSpeedSmoothValue = 0.5;
float gpsCourseSmoothValue = 0.5;

#define MAX_GPS_ANGLE_CORRECTION 200
//#define MAX_NAVIGATON_SPEED 400  // m/s * 100 // 3 m/s = 10.8km/h
#define MAX_NAVIGATON_SPEED 60.0  // m/s * 100 // 3 m/s = 10.8km/h

void processPositionCorrection() {
  
  float derivateDistanceX = ((float)currentPosition.longitude - (float)previousPosition.longitude) * 0.649876;
  float derivateDistanceY = ((float)currentPosition.latitude - (float)previousPosition.latitude) * 1.113195;
  float derivateDistance = sqrt(sq(derivateDistanceY) + sq(derivateDistanceX));
  
  float distanceX = ((float)positionToReach.longitude - (float)currentPosition.longitude) * 0.649876;
  float distanceY = ((float)positionToReach.latitude - (float)currentPosition.latitude) * 1.113195;
  float distance = sqrt(sq(distanceY) + sq(distanceX));
  
  gpsLaggedSpeed = gpsLaggedSpeed * (gpsSpeedSmoothValue) + derivateDistance * (1-gpsSpeedSmoothValue);
  if (derivateDistanceX != 0 || derivateDistanceY != 0) {
    float tmp = degrees(atan2(derivateDistanceX, derivateDistanceY));
      if (tmp < 0) {
//        tmp += 360; // jakub fix, logic but... I had weird behavior, I need more investigation!
        tmp += radians(360);
      }
      gpsLaggedCourse = (int)((float)gpsLaggedCourse*(gpsCourseSmoothValue) + tmp*100*(1-gpsCourseSmoothValue));
  }
  
  float angleToWaypoint = atan2(distanceX, distanceY);
  float courseRads = radians(gpsLaggedCourse/100);
  
//  float azimuth = getAbsoluteHeading();
  float azimuth = kinematicsAngle[ZAXIS];
  float currentSpeedCmPerSecRoll = sin(courseRads-azimuth)*gpsLaggedSpeed; 
  float currentSpeedCmPerSecPitch = cos(courseRads-azimuth)*gpsLaggedSpeed;
    
  float angle = angleToWaypoint-azimuth;
  float tmpsin = sin(angle);
  float tmpcos = cos(angle);
    
  float maxSpeedRoll = (MAX_NAVIGATON_SPEED*tmpsin*((float)distance)); 
  float maxSpeedPitch = (MAX_NAVIGATON_SPEED*tmpcos*((float)distance));
  maxSpeedRoll = constrain(maxSpeedRoll, -MAX_NAVIGATON_SPEED, MAX_NAVIGATON_SPEED);
  maxSpeedPitch = constrain(maxSpeedPitch, -MAX_NAVIGATON_SPEED, MAX_NAVIGATON_SPEED);

  gpsRollAxisCorrection = updatePID(maxSpeedRoll, currentSpeedCmPerSecRoll, &PID[GPSROLL_PID_IDX]);
  gpsPitchAxisCorrection = updatePID(maxSpeedPitch, currentSpeedCmPerSecPitch , &PID[GPSPITCH_PID_IDX]);
  
  gpsRollAxisCorrection = constrain(gpsRollAxisCorrection, -MAX_GPS_ANGLE_CORRECTION, MAX_GPS_ANGLE_CORRECTION);
  gpsPitchAxisCorrection = constrain(gpsPitchAxisCorrection, -MAX_GPS_ANGLE_CORRECTION, MAX_GPS_ANGLE_CORRECTION);

  previousPosition.latitude = currentPosition.latitude;
  previousPosition.longitude = currentPosition.longitude;
}



#endif

