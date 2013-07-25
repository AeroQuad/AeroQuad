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

double rad(double degrees) {
  return degrees * 0.0174532925;
}

double deg(double radians) {
  return radians * 57.2957795;
}

double adjustHeading(double currentHeading, double desiredHeading) {
  if ((desiredHeading < -90.0) && (currentHeading > (desiredHeading + 180.0)))
    return currentHeading -= 360.0;
  if ((desiredHeading > 90.0) && ((desiredHeading - 180.0) > currentHeading))
    return currentHeading += 360.0;
  if ((desiredHeading > -90.0) && ((desiredHeading + 180.0) < currentHeading))
    return currentHeading -= 360.0;
  if ((desiredHeading < 90.0) && (currentHeading < (desiredHeading - 180.0)))
    return currentHeading += 360.0;
  return currentHeading;
}

void positionVector(double *vector, GeodeticPosition position) {
  double lat = (double)position.latitude * GPS2RAD;
  double lon = (double)position.longitude * GPS2RAD;
  vector[0] = cos(lat) * cos(lon);
  vector[1] = cos(lat) * sin(lon);
  vector[2] = sin(lat);
}

#define MIN_DISTANCE_TO_REACHED 2000
/**
 * Evaluate the position to reach depending of the state of the mission
 */
bool evaluateMissionPositionToReach() { // TODO: rename this

  if (waypointIndex == -1) { // if mission have not been started
    waypointIndex++;
  }

  /*
  if (waypointIndex < MAX_WAYPOINTS && distanceToDestination < MIN_DISTANCE_TO_REACHED) {
    waypointIndex++;
  }

  if (waypointIndex >= MAX_WAYPOINTS ||
      waypoint[waypointIndex].altitude == GPS_INVALID_ALTITUDE) { // if mission is completed, last step is to go home 2147483647 == invalid altitude

    missionPositionToReach.latitude = homePosition.latitude;
    missionPositionToReach.longitude = homePosition.longitude;
    missionPositionToReach.altitude = homePosition.altitude;
    return true; // finished route
  }
  else {
*/
    missionPositionToReach.latitude = waypoint[waypointIndex].latitude;
    missionPositionToReach.longitude = waypoint[waypointIndex].longitude;
    missionPositionToReach.altitude = (waypoint[waypointIndex].altitude/100);

    fromWaypoint = waypoint[waypointIndex];
    toWaypoint = waypoint[waypointIndex+1];
    positionVector(fromVector, fromWaypoint);
    positionVector(toVector, toWaypoint);
    vectorCrossProductDbl(normalVector, fromVector, toVector);
    vectorNormalize(normalVector);
    negNormalVector[0] = -normalVector[0];
    negNormalVector[1] = -normalVector[1];
    negNormalVector[2] = -normalVector[2];

    if (missionPositionToReach.altitude > 2000.0) {
      missionPositionToReach.altitude = 2000.0; // fix max altitude to 2 km
    }
    return false;
  //}
}

void loadNewRoute() {
  waypointIndex = -1;
  evaluateMissionPositionToReach();
}

/**
 * Process navigation
 */
void processNavigation() {
  // Convert lat/lon to ECEF
  positionVector(presentPosition, currentPosition);

  // Calculate track angle error
  vectorCrossProductDbl(presentPositionEast, zVector, presentPosition);
  vectorNormalize(presentPositionEast);
  vectorCrossProductDbl(presentPositionNorth, presentPosition, presentPositionEast);
  vectorNormalize(presentPositionNorth);
  desiredHeading = deg(atan2(vectorDotProductDbl(normalVector, presentPositionNorth), vectorDotProductDbl(negNormalVector, presentPositionEast)));
  currentHeading = adjustHeading(heading, desiredHeading);
  // We'll use trackAngleError when we have velocity information, use desiredHeading for now
  trackAngleError = desiredHeading - currentHeading;

  // Calculate cross track error
  vectorCrossProductDbl(normalPerpendicularVector, presentPosition, normalVector);
  vectorNormalize(normalPerpendicularVector);
  vectorCrossProductDbl(alongPathVector, normalVector, normalPerpendicularVector);
  vectorNormalize(alongPathVector);
  crossTrackError = earthRadius * atan2(vectorDotProductDbl(negNormalVector, presentPosition), vectorDotProductDbl(alongPathVector, presentPosition));

  // Calculate distance to next waypoint
  vectorCrossProductDbl(normalRangeVector, presentPosition, toVector);
  vectorNormalize(normalRangeVector);
  vectorCrossProductDbl(rangeVector, toVector, normalRangeVector);
  vectorNormalize(rangeVector);
  distanceToNextWaypoint = earthRadius * atan2(vectorDotProductDbl(rangeVector, presentPosition), vectorDotProductDbl(presentPosition, toVector));

  if (distanceToNextWaypoint < waypointCaptureDistance) {
    boolean finishedRoute = evaluateMissionPositionToReach();
    if (finishedRoute)
      positionHoldState = ON;
  }

  crossTrack = constrain(crossTrackFactor * crossTrackError, -MAXCROSSTRACKANGLE, MAXCROSSTRACKANGLE);
  groundTrackHeading = desiredHeading + crossTrack;

  // not used
  gpsRollAxisCorrection = 0;
  gpsPitchAxisCorrection = 0;
  gpsYawAxisCorrection = 0;
}

void processPositionHold()
{
  // Need to figure out how to set positionHoldPointToReach
  gpsRollAxisCorrection = (positionHoldPointToReach.longitude - currentPosition.longitude) * cos(trueNorthHeading) - (missionPositionToReach.latitude - currentPosition.latitude) * sin(trueNorthHeading);
  gpsRollAxisCorrection *= positionHoldFactor;
  gpsPitchAxisCorrection = (positionHoldPointToReach.longitude - currentPosition.longitude) * sin(trueNorthHeading) + (missionPositionToReach.latitude - currentPosition.latitude) * cos(heading);
  gpsPitchAxisCorrection *= positionHoldFactor;
  gpsYawAxisCorrection = 0;
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
#endif

#endif











