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
      // Set the magnetometer declination when we get the home position set
      setDeclinationLocation(currentPosition.latitude,currentPosition.longitude);
    }  
  }
}

boolean haveMission() {
  return missionNbPoint != 0;
}




#define GPS_SPEED_SMOOTH_VALUE 0.5
#define GPS_COURSE_SMOOTH_VALUE 0.5

GeodeticPosition previousPosition;
float gpsLaggedSpeed = 0.0;
float gpsLaggedCourse = 0.0;
float currentSpeedCmPerSecRoll = 0.0; 
float currentSpeedCmPerSecPitch = 0.0;

void computeCurrentSpeedInCmPerSec() {

  float derivateDistanceX = ((float)currentPosition.longitude - (float)previousPosition.longitude) * 0.649876;
  float derivateDistanceY = ((float)currentPosition.latitude - (float)previousPosition.latitude) * 1.113195;
  float derivateDistance = sqrt(sq(derivateDistanceY) + sq(derivateDistanceX));

  gpsLaggedSpeed = gpsLaggedSpeed * (GPS_SPEED_SMOOTH_VALUE) + derivateDistance * (1-GPS_SPEED_SMOOTH_VALUE);
  if (derivateDistanceX != 0 || derivateDistanceY != 0) {
    float tmp = degrees(atan2(derivateDistanceX, derivateDistanceY));
      if (tmp < 0) {
//        tmp += 360; // jakub fix, logic but... I had weird behavior, I need more investigation!
        tmp += radians(360);
      }
      gpsLaggedCourse = (int)((float)gpsLaggedCourse*(GPS_COURSE_SMOOTH_VALUE) + tmp*100*(1-GPS_COURSE_SMOOTH_VALUE));
  }

  float courseRads = radians(gpsLaggedCourse/100);
  currentSpeedCmPerSecRoll = sin(courseRads-trueNorthHeading)*gpsLaggedSpeed; 
  currentSpeedCmPerSecPitch = cos(courseRads-trueNorthHeading)*gpsLaggedSpeed;
  
  previousPosition.latitude = currentPosition.latitude;
  previousPosition.longitude = currentPosition.longitude;
}
  

#define MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION 200.0
#define MAX_NAVIGATION_ANGLE_CORRECTION 200.0
#define NAVIGATION_SPEED 400.0  // m/s * 100 // 3 m/s = 10.8km/h
#define POSITION_HOLD_SPEED 60.0  
  
float maxSpeedToDestination = POSITION_HOLD_SPEED;
float maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;

// use to compute the heading in position hold state
float distanceX = 0.0;
float distanceY = 0.0;

void computeDistanceToDestination() {
  
  distanceX = ((float)positionToReach.longitude - (float)currentPosition.longitude) * 0.649876;
  distanceY = ((float)positionToReach.latitude - (float)currentPosition.latitude) * 1.113195;
  gpsDistanceToDestination  = sqrt(sq(distanceY) + sq(distanceX));

}


void processPositionCorrection() {
  
  // compute current speed in cm per sec
  computeCurrentSpeedInCmPerSec();
  
  // compute distance to destination
  computeDistanceToDestination();
  
  
  
  float angleToWaypoint = atan2(distanceX, distanceY);
    
  float angle = angleToWaypoint-trueNorthHeading;
  float tmpsin = sin(angle);
  float tmpcos = cos(angle);
    
  float maxSpeedRoll = (maxSpeedToDestination*tmpsin*((float)gpsDistanceToDestination)); 
  float maxSpeedPitch = (maxSpeedToDestination*tmpcos*((float)gpsDistanceToDestination));
  maxSpeedRoll = constrain(maxSpeedRoll, -maxSpeedToDestination, maxSpeedToDestination);
  maxSpeedPitch = constrain(maxSpeedPitch, -maxSpeedToDestination, maxSpeedToDestination);

  gpsRollAxisCorrection = updatePID(maxSpeedRoll, currentSpeedCmPerSecRoll, &PID[GPSROLL_PID_IDX]);
  gpsPitchAxisCorrection = updatePID(maxSpeedPitch, currentSpeedCmPerSecPitch , &PID[GPSPITCH_PID_IDX]);
  
  gpsRollAxisCorrection = constrain(gpsRollAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
  gpsPitchAxisCorrection = constrain(gpsPitchAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
}



#endif

