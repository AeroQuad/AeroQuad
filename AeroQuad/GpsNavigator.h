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

// little wait to have a more precise fix of the current position since it's called after the first gps fix
#define MIN_NB_GPS_READ_TO_INIT_HOME 15  
byte countToInitHome = 0;

unsigned long previousFixTime = 0;

static boolean isNewGpsPosition() {
  return (haveAGpsLock() && (previousFixTime != getGpsFixTime()));
}

static void clearNewGpsPosition() {
  previousFixTime = getGpsFixTime();
}
  
boolean isHomeBaseInitialized() {
  return homePosition.latitude != GPS_INVALID_ANGLE;
}

void initHomeBase() {
  if (isNewGpsPosition()) {
    clearNewGpsPosition();
    if (countToInitHome < MIN_NB_GPS_READ_TO_INIT_HOME) {
      countToInitHome++;
    }
    else {
      homePosition.latitude = currentPosition.latitude;
      homePosition.longitude = currentPosition.longitude;
      homePosition.altitude = DEFAULT_HOME_ALTITUDE;  
      // Set the magnetometer declination when we get the home position set
      setDeclinationLocation(currentPosition.latitude,currentPosition.longitude);
      // Set reference location for Equirectangular projection used for coordinates
      setProjectionLocation(currentPosition);
      

      #if defined UseGPSNavigator
        evaluateMissionPositionToReach();
      #else
        missionPositionToReach.latitude = homePosition.latitude;
        missionPositionToReach.longitude = homePosition.longitude;
        missionPositionToReach.altitude = homePosition.altitude;
      #endif
    }  
  }
}


#if defined UseGPSNavigator

  /*
    Because we are using lat and lon to do our distance errors here's a quick chart:
    100 	= 1m
    1000 	= 11m	 = 36 feet
    1800 	= 19.80m = 60 feet
    3000 	= 33m
    10000       = 111m
  */
  
  #define MIN_DISTANCE_TO_REACHED 3000

  #define GPS_SPEED_SMOOTH_VALUE 0.5
  #define GPS_COURSE_SMOOTH_VALUE 0.5
  
  #define MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION 300.0
  #define POSITION_HOLD_SPEED 60.0  
  #define MAX_NAVIGATION_ANGLE_CORRECTION 300.0
  #define NAVIGATION_SPEED 250.0  // m/s * 100 // 3 m/s = 10.8km/h
  
  #define MAX_YAW_AXIS_CORRECTION 200.0  
    
  GeodeticPosition previousPosition = GPS_INVALID_POSITION;
  float gpsLaggedSpeed = 0.0;
  float gpsLaggedCourse = 0.0;
  float currentSpeedCmPerSecRoll = 0.0; 
  float currentSpeedCmPerSecPitch = 0.0;
  
  float distanceX = 0.0;
  float distanceY = 0.0;
  float angleToWaypoint = 0.0;
  
  float maxSpeedToDestination = POSITION_HOLD_SPEED;
  float maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;
  
  #if defined AltitudeHoldRangeFinder
    boolean altitudeProximityAlert = false;
    byte altitudeProximityAlertSecurityCounter = 0;
  #endif


  /** 
   * @return true if there is a mission to execute
   */
  boolean haveMission() {
    return missionNbPoint != 0;
  }


  /**
   * Evalutate the position to reach depending of the state of the mission 
   */
  void evaluateMissionPositionToReach() {

    if (waypointIndex == -1) { // if mission have not been started
      waypointIndex++;
    }
    
    if (waypointIndex < MAX_WAYPOINTS && gpsDistanceToDestination < MIN_DISTANCE_TO_REACHED) {
      waypointIndex++;
    }
    
    if (waypointIndex >= MAX_WAYPOINTS || 
        waypoint[waypointIndex].altitude == GPS_INVALID_ALTITUDE) { // if mission is completed, last step is to go home 2147483647 == invalid altitude

      missionPositionToReach.latitude = homePosition.latitude;
      missionPositionToReach.longitude = homePosition.longitude;
      missionPositionToReach.altitude = homePosition.altitude; 
    }
    else {
      
      missionPositionToReach.latitude = waypoint[waypointIndex].latitude;
      missionPositionToReach.longitude = waypoint[waypointIndex].longitude;
      missionPositionToReach.altitude = (waypoint[waypointIndex].altitude/100);

      if (missionPositionToReach.altitude > 2000.0) {
        missionPositionToReach.altitude = 2000.0; // fix max altitude to 2 km
      }
    }
  }

  /** 
   * Compute the distance to the destination, point to reach
   * @result is gpsDistanceToDestination
   */
  void computeDistanceToDestination(GeodeticPosition destination) {
    
    distanceX = (float)(destination.longitude - currentPosition.longitude) * cosLatitude * 1.113195;
    distanceY = (float)(destination.latitude - currentPosition.latitude) * 1.113195;
    gpsDistanceToDestination  = sqrt(sq(distanceY) + sq(distanceX));
  }

  /**
   * Compute the current craft speed in cm per sec
   * @result are currentSpeedCmPerSecPitch and currentSpeedCmPerSecRoll
   */
  void computeCurrentSpeedInCmPerSec() {
  
    float derivateDistanceX = (float)(currentPosition.longitude - previousPosition.longitude) * cosLatitude * 1.113195;
    float derivateDistanceY = (float)(currentPosition.latitude - previousPosition.latitude) * 1.113195;
    float derivateDistance = sqrt(sq(derivateDistanceY) + sq(derivateDistanceX));
  
    gpsLaggedSpeed = gpsLaggedSpeed * (GPS_SPEED_SMOOTH_VALUE) + derivateDistance * (1-GPS_SPEED_SMOOTH_VALUE);
    if (derivateDistanceX != 0 || derivateDistanceY != 0) {
      float tmp = degrees(atan2(derivateDistanceX, derivateDistanceY));
        if (tmp < 0) {
          tmp += 360; 
        }
        gpsLaggedCourse = (int)((float)gpsLaggedCourse*(GPS_COURSE_SMOOTH_VALUE) + tmp*100*(1-GPS_COURSE_SMOOTH_VALUE));
    }
  
    float courseRads = radians(gpsLaggedCourse/100);
    currentSpeedCmPerSecRoll = sin(courseRads-trueNorthHeading)*gpsLaggedSpeed; 
    currentSpeedCmPerSecPitch = cos(courseRads-trueNorthHeading)*gpsLaggedSpeed;
    
    previousPosition.latitude = currentPosition.latitude;
    previousPosition.longitude = currentPosition.longitude;
  }
    
  
  /**
   * Evaluate the flight behavior to adopt depending of the distance to the point to reach
   */
  void evaluateFlightBehaviorFromDistance() {

    if (gpsDistanceToDestination < MIN_DISTANCE_TO_REACHED) {  // position hold

      maxSpeedToDestination = POSITION_HOLD_SPEED;
      maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;
    }
    else { // navigate

      maxSpeedToDestination = NAVIGATION_SPEED;
      maxCraftAngleCorrection = MAX_NAVIGATION_ANGLE_CORRECTION;
    }
  }
  
  /**
   * compute craft angle in roll/pitch to adopt to navigate to the point to reach
   * @result are gpsRollAxisCorrection and gpsPitchAxisCorrection use in flight control processor
   */
  void computeRollPitchCraftAxisCorrection() {
    
    angleToWaypoint = atan2(distanceX, distanceY)-trueNorthHeading;
    float tmpsin = sin(angleToWaypoint);
    float tmpcos = cos(angleToWaypoint);
    
    float maxSpeedRoll = (maxSpeedToDestination*tmpsin*((float)gpsDistanceToDestination)); 
    float maxSpeedPitch = (maxSpeedToDestination*tmpcos*((float)gpsDistanceToDestination));
    maxSpeedRoll = constrain(maxSpeedRoll, -maxSpeedToDestination, maxSpeedToDestination);
    maxSpeedPitch = constrain(maxSpeedPitch, -maxSpeedToDestination, maxSpeedToDestination);
    
//    gpsRollAxisCorrection = updatePID(maxSpeedRoll, currentSpeedCmPerSecRoll, &PID[GPSROLL_PID_IDX]);
//    gpsPitchAxisCorrection = updatePID(maxSpeedPitch, currentSpeedCmPerSecPitch, &PID[GPSPITCH_PID_IDX]);
    gpsRollAxisCorrection = filterSmooth(updatePID(maxSpeedRoll, currentSpeedCmPerSecRoll, &PID[GPSROLL_PID_IDX]), gpsRollAxisCorrection, 0.5);
    gpsPitchAxisCorrection = filterSmooth(updatePID(maxSpeedPitch, currentSpeedCmPerSecPitch, &PID[GPSPITCH_PID_IDX]), gpsPitchAxisCorrection, 0.5);

    gpsRollAxisCorrection = constrain(gpsRollAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
    gpsPitchAxisCorrection = constrain(gpsPitchAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
  }
  
  
  /**
   * Evaluate altitude to reach, if we use the range finder, we use it as altitude proximity alert
   * to increase the current point to reach altitude
   */
  void evaluateAltitudeCorrection() {
//    #if defined AltitudeHoldRangeFinder
//      // if this is true, we are too near the ground to perform navigation, then, make current alt hold target +25m
//      if (sonarAltitudeToHoldTarget != INVALID_RANGE) { 
//        if (!altitudeProximityAlert) {
//          sonarAltitudeToHoldTarget += 2;
//          missionPositionToReach.altitude += 2;
//          altitudeProximityAlert = true;
//        }
//      }
//
//      if (altitudeProximityAlert && altitudeProximityAlertSecurityCounter <= 10) {
//        altitudeProximityAlertSecurityCounter++;
//      }
//      else {
//        altitudeProximityAlertSecurityCounter = 0;
//        altitudeProximityAlert = false;
//      }
//
//    #endif
    #if defined AltitudeHoldRangeFinder
      if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
        sonarAltitudeToHoldTarget += 0.05;
      }
    #endif
    baroAltitudeToHoldTarget = missionPositionToReach.altitude;
  }
  
  /**
   * In navigation mode, we want the craft headed to the target, so this will 
   * compute the heading correction to have
   */
  void computeHeadingCorrection() {
    
    float correctionAngle = angleToWaypoint;
    if (correctionAngle > PI) {
      correctionAngle = fmod(correctionAngle,PI) - PI;
    }

    gpsYawAxisCorrection = -updatePID(0.0, correctionAngle, &PID[GPSYAW_PID_IDX]);
    gpsYawAxisCorrection = constrain(gpsYawAxisCorrection, -MAX_YAW_AXIS_CORRECTION, MAX_YAW_AXIS_CORRECTION);
  }

  /**
   * Process position hold
   */
  void processPositionHold() {
    
    if (!isNewGpsPosition()) {
      return;
    }
    
    computeDistanceToDestination(positionHoldPointToReach);
    
    // evaluate the flight behavior to adopt
    maxSpeedToDestination = POSITION_HOLD_SPEED;
    maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;

    computeRollPitchCraftAxisCorrection();

    gpsYawAxisCorrection = 0;  
    
    clearNewGpsPosition();
  }
  
    /** 
   * Process navigation
   */
  void processNavigation() {
    
    if (!isNewGpsPosition()) {
      return;
    }
    
    // evaluate if we need to switch to another mission possition point
    evaluateMissionPositionToReach();
    
    computeDistanceToDestination(missionPositionToReach);

    // evaluate the flight behavior to adopt
    evaluateFlightBehaviorFromDistance();

    computeRollPitchCraftAxisCorrection();
    
    evaluateAltitudeCorrection();    

    computeHeadingCorrection();
    
    clearNewGpsPosition();
  }

  
  /**
   * Compute everything need to make adjustment to the craft attitude to go to the point to reach
   */
  void processGpsNavigation() {

    if (haveAGpsLock()) {
      
      if (isNewGpsPosition()) {
        computeCurrentSpeedInCmPerSec();
      }
      
//      if (navigationState == ON) {
//        processNavigation();
//      }
//      else 
      if (positionHoldState == ON ) {
        processPositionHold();
      }
      
      clearNewGpsPosition();

    }
  }
#endif  // #define UseGPSNavigator


#endif
