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

boolean haveNewGpsPosition() {
  return (haveAGpsLock() && (previousFixTime != getGpsFixTime()));
}

void clearNewGpsPosition() {
  previousFixTime = getGpsFixTime();
}
  
boolean isHomeBaseInitialized() {
  return homePosition.latitude != GPS_INVALID_ANGLE;
}

void initHomeBase() {
  if (haveNewGpsPosition()) {
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
  
  #define MIN_DISTANCE_TO_REACHED 2000

  #define MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION 300.0
  #define POSITION_HOLD_SPEED 300.0  
  #define MAX_NAVIGATION_ANGLE_CORRECTION 300.0
  #define NAVIGATION_SPEED 600.0 
  
  #define MAX_YAW_AXIS_CORRECTION 200.0  
    
  long readingDelay = 0;  
  long estimatedDelay = 0;
  unsigned long previousEstimationTime = 0;
  unsigned long previousReadingTime = 0;
  int latitudeMovement = 0;
  int longitudeMovement = 0;
  GeodeticPosition previousPosition = GPS_INVALID_POSITION;
  GeodeticPosition estimatedPosition = GPS_INVALID_POSITION;
  GeodeticPosition estimatedPreviousPosition = GPS_INVALID_POSITION;
  float currentSpeedRoll = 0.0; 
  float currentSpeedPitch = 0.0;
  
  float distanceToDestinationX = 0.0;
  float distanceToDestinationY = 0.0;
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
    
    if (waypointIndex < MAX_WAYPOINTS && distanceToDestination < MIN_DISTANCE_TO_REACHED) {
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


  void computeNewPosition() {
    
    unsigned long time = micros();
    readingDelay = time - previousReadingTime;
    previousReadingTime = time;
    estimatedDelay = time - previousEstimationTime;
    previousEstimationTime = time;
    
    latitudeMovement = currentPosition.latitude - previousPosition.latitude;
    longitudeMovement = currentPosition.longitude - previousPosition.longitude;
    
    previousPosition.latitude  = currentPosition.latitude;
    previousPosition.longitude = currentPosition.longitude;
    previousPosition.altitude  = currentPosition.altitude;
    
    estimatedPreviousPosition.latitude = estimatedPosition.latitude;
    estimatedPreviousPosition.longitude = estimatedPosition.longitude;
    
    estimatedPosition.latitude  = currentPosition.latitude;
    estimatedPosition.longitude = currentPosition.longitude;
    estimatedPosition.altitude  = currentPosition.altitude;
  }
  
  void computeEstimatedPosition() {
    
    unsigned long time = micros();
    estimatedDelay = time - previousEstimationTime;
    previousEstimationTime = time;
    
    estimatedPreviousPosition.latitude = estimatedPosition.latitude;
    estimatedPreviousPosition.longitude = estimatedPosition.longitude;
    
    estimatedPosition.latitude += (latitudeMovement / (readingDelay / estimatedDelay));
    estimatedPosition.longitude += (longitudeMovement / (readingDelay / estimatedDelay));
  }



  /** 
   * Compute the distance to the destination, point to reach
   * @result is distanceToDestination
   */
  void computeDistanceToDestination(GeodeticPosition destination) {
    
    distanceToDestinationX = (float)(destination.longitude - estimatedPosition.longitude) * cosLatitude * 1.113195;
    distanceToDestinationY = (float)(destination.latitude  - estimatedPosition.latitude) * 1.113195;
    distanceToDestination  = sqrt(sq(distanceToDestinationY) + sq(distanceToDestinationX));
  }

  /**
   * Compute the current craft speed in cm per sec
   * @result are currentSpeedPitch and currentSpeedRoll
   */
  void computeCurrentSpeed() {
  
    float currentSpeedX = (float)(estimatedPosition.longitude - estimatedPreviousPosition.longitude) * cosLatitude * 1.113195;
    float currentSpeedY = (float)(estimatedPosition.latitude - estimatedPreviousPosition.latitude) * 1.113195;
    float currentSpeed = sqrt(sq(currentSpeedY) + sq(currentSpeedX));
    
    currentSpeedX = currentSpeedX * (100000 / estimatedDelay); // normalized to about 5hz
    currentSpeedY = currentSpeedY * (100000 / estimatedDelay); 
    currentSpeed = currentSpeed * (100000 / estimatedDelay); 
  
    float tmp = degrees(atan2(currentSpeedX, currentSpeedY));
    if (tmp < 0) {
      tmp += 360; 
    }

    float courseRads = radians(tmp);
    currentSpeedRoll = (sin(courseRads-trueNorthHeading)*currentSpeed); 
    currentSpeedPitch = (cos(courseRads-trueNorthHeading)*currentSpeed);
  }
    
  /**
   * compute craft angle in roll/pitch to adopt to navigate to the point to reach
   * @result are gpsRollAxisCorrection and gpsPitchAxisCorrection use in flight control processor
   */
  void computeRollPitchCraftAxisCorrection() {
    
    angleToWaypoint = atan2(distanceToDestinationX, distanceToDestinationY)-trueNorthHeading;
    float tmpsin = sin(angleToWaypoint);
    float tmpcos = cos(angleToWaypoint);
    
    float rollSpeedDesired = ((maxSpeedToDestination*tmpsin)*(float)distanceToDestination)/1000; 
    float pitchSpeedDesired = ((maxSpeedToDestination*tmpcos)*(float)distanceToDestination)/1000;
    rollSpeedDesired = constrain(rollSpeedDesired, -maxSpeedToDestination, maxSpeedToDestination);
    pitchSpeedDesired = constrain(pitchSpeedDesired, -maxSpeedToDestination, maxSpeedToDestination);
    
    int tempGpsRollAxisCorrection = updatePID(rollSpeedDesired, currentSpeedRoll, &PID[GPSROLL_PID_IDX]);
    int tempGpsPitchAxisCorrection = updatePID(pitchSpeedDesired, currentSpeedPitch, &PID[GPSPITCH_PID_IDX]);

    if (tempGpsRollAxisCorrection >= gpsRollAxisCorrection) {
      gpsRollAxisCorrection += 1;
    }
    else {
      gpsRollAxisCorrection -= 1;
    }
    if (tempGpsPitchAxisCorrection >= gpsPitchAxisCorrection) {
      gpsPitchAxisCorrection += 1;
    }
    else {
      gpsPitchAxisCorrection -= 1;
    }
    
    gpsRollAxisCorrection = constrain(gpsRollAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
    gpsPitchAxisCorrection = constrain(gpsPitchAxisCorrection, -maxCraftAngleCorrection, maxCraftAngleCorrection);
    
//    Serial.print(gpsData.sats);Serial.print(" ");Serial.print(distanceToDestination);Serial.print(" ");
//    Serial.print(rollSpeedDesired);Serial.print(",");Serial.print(pitchSpeedDesired);Serial.print(" ");
//    Serial.print(currentSpeedRoll);Serial.print(",");Serial.print(currentSpeedPitch);Serial.print(" ");
//    Serial.print(gpsRollAxisCorrection);Serial.print(",");Serial.print(gpsPitchAxisCorrection);Serial.print(" ");
//    Serial.println();
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
//    #if defined AltitudeHoldRangeFinder
//      if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
//        sonarAltitudeToHoldTarget += 0.05;
//      }
//    #endif
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
    
    if (haveNewGpsPosition()) {
      computeNewPosition();
      clearNewGpsPosition();
    }
    else {
      computeEstimatedPosition();
    }    
    
    computeCurrentSpeed();
    
    computeDistanceToDestination(positionHoldPointToReach);
    
    // evaluate the flight behavior to adopt
    maxSpeedToDestination = POSITION_HOLD_SPEED;
    maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;

    computeRollPitchCraftAxisCorrection();

    gpsYawAxisCorrection = 0;  
  }
  
    /** 
   * Process navigation
   */
  void processNavigation() {
    
    if (distanceToDestination < MIN_DISTANCE_TO_REACHED) {
      processPositionHold();
      evaluateMissionPositionToReach();
      return;
    }
    else if (haveNewGpsPosition()) {
      computeNewPosition();
      clearNewGpsPosition();
    }
    else {
      return;
    }
    
    computeCurrentSpeed();
    
    // evaluate if we need to switch to another mission possition point
    evaluateMissionPositionToReach();
    
    computeDistanceToDestination(missionPositionToReach);

    maxSpeedToDestination = NAVIGATION_SPEED;
    maxCraftAngleCorrection = MAX_NAVIGATION_ANGLE_CORRECTION;

    computeRollPitchCraftAxisCorrection();
    
    evaluateAltitudeCorrection();    

    computeHeadingCorrection();
  }

  
  /**
   * Compute everything need to make adjustment to the craft attitude to go to the point to reach
   */
  void processGpsNavigation() {

    if (haveAGpsLock()) {
      
      if (navigationState == ON) {
        processNavigation();
      }
      else if (positionHoldState == ON ) {
        processPositionHold();
      }
    }
  }
#endif  // #define UseGPSNavigator


#endif











