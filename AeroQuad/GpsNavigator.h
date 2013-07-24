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

float rad(float degrees) {
  return degrees * 0.0174532925;
}

float deg(float radians) {
  return radians * 57.2957795;
}

float adjustHeading(float currentHeading, float desiredHeading) {
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

void positionVector(float *vector, GeodeticPosition position) {
  float lat = rad((float)position.latitude/10000000.0);
  float lon = rad((float)position.longitude/10000000.0);
  vector[0] = cos(lat) * cos(lon);
  vector[1] = cos(lat) * sin(lon);
  vector[2] = sin(lat);
}

/*
float calculateDistance(GeodeticPosition fromWaypoint, GeodeticPosition toWaypoint) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  float dLat = rad((toWaypoint.latitude - fromWaypoint.latitude)/10000000.0);
  float dLon = rad((toWaypoint.longitude - fromWaypoint.longitude)/10000000.0);
  float lat1 = rad(fromWaypoint.latitude/10000000.0);
  float lat2 = rad(toWaypoint.latitude/10000000.0);
  float a = sin(dLat/2.0) * sin(dLat/2.0) + sin(dLon/2.0) * sin(dLon/2.0) * cos(lat1) * cos(lat2);
  return earthRadius * 2.0 * atan2(sqrt(a), sqrt(1-a));
}

double calculateDistance(GeodeticPosition currentWP, GeodeticPosition nextWP) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  // Use this for double precision values
  double lat1 = rad(currentWP.latitude/10000000.0);
  double lat2 = rad(nextWP.latitude/10000000.0);
  double lon1 = rad(currentWP.longitude/10000000.0);
  double lon2 = rad(nextWP.longitude/10000000.0);
  return acos((sin(lat1)*sin(lat2))+(cos(lat1)*cos(lat2)*cos(lon2-lon1)))*earthRadius;
}

float calculateBearing(GeodeticPosition currentWP, GeodeticPosition nextWP) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  float lat1 = rad(currentWP.latitude/10000000.0);
  float lat2 = rad(nextWP.latitude/10000000.0);
  float lon1 = rad(currentWP.longitude/10000000.0);
  float lon2 = rad(nextWP.longitude/10000000.0);
  float y = cos(lon2-lon1) * cos(lat2);
  float x = (cos(lat1)*sin(lat2)) - (sin(lat1)*cos(lat2)*cos(lon2-lon1));
  return deg(atan2(y,x));
}

float calculateCrossTrack(float distanceFromStartToPosition, float bearingFromStartToPosition, float bearingFromStartToEnd) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  return earthRadius * asin(sin(distanceFromStartToPosition/earthRadius) * sin(rad(bearingFromStartToPosition-bearingFromStartToEnd)));
}

float calculateAlongPathDistance(float distanceFromStartToPosition, float crossTrackDistance) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  return earthRadius * acos(cos(distanceFromStartToPosition/earthRadius) / cos(crossTrackDistance/earthRadius));
}
*/

/**
 * @return true if there is a mission to execute
 */
//boolean haveMission() {
//  return missionNbPoint != 0;
//}

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

    fromWaypoint = waypoint[0];
    toWaypoint = waypoint[1];


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
  positionVector(fromVector, fromWaypoint);
  positionVector(toVector, toWaypoint);
  positionVector(presentPosition, currentPosition);

  // Calculate track angle error
  vectorCrossProduct(presentPositionEast, zVector, presentPosition);
  vectorNormalize(presentPositionEast);
  vectorCrossProduct(presentPositionNorth, presentPosition, presentPositionEast);
  vectorNormalize(presentPositionNorth);
  vectorCrossProduct(normalVector, fromVector, toVector);
  vectorNormalize(normalVector);
  negNormalVector[0] = -normalVector[0];
  negNormalVector[1] = -normalVector[1];
  negNormalVector[2] = -normalVector[2];
  desiredHeading = deg(atan2(vectorDotProduct(3, normalVector, presentPositionNorth), vectorDotProduct(3, negNormalVector, presentPositionEast)));
  currentHeading = adjustHeading(heading, desiredHeading);
  // We'll use trackAngleError when we have velocity information, use desiredHeading for now
  trackAngleError = desiredHeading - currentHeading;

  // Calculate cross track error
  vectorCrossProduct(normalPerpendicularVector, presentPosition, normalVector);
  vectorNormalize(normalPerpendicularVector);
  vectorCrossProduct(alongPathVector, normalVector, normalPerpendicularVector);
  vectorNormalize(alongPathVector);
  crossTrackError = earthRadius * atan2(vectorDotProduct(3, negNormalVector, presentPosition), vectorDotProduct(3, alongPathVector, presentPosition));
  //distanceFromStartToPosition = calculateDistance(fromWaypoint, currentPosition);
  //bearingFromStartToPosition = calculateBearing(fromWaypoint, currentPosition);
  //bearingFromStartToNextWP = calculateBearing(fromWaypoint, toWaypoint);
  //crossTrackError = calculateCrossTrack(distanceFromStartToPosition, bearingFromStartToPosition, bearingFromStartToNextWP);
  //alongPathDistance = calculateAlongPathDistance(distanceFromStartToPosition, crossTrackError);

  // Calculate distance to next waypoint
  vectorCrossProduct(normalRangeVector, presentPosition, toVector);
  vectorNormalize(normalRangeVector);
  vectorCrossProduct(rangeVector, toVector, normalRangeVector);
  vectorNormalize(rangeVector);
  distanceToNextWaypoint = earthRadius * atan2(vectorDotProduct(3, rangeVector, presentPosition), vectorDotProduct(3, presentPosition, toVector));

  if (distanceToNextWaypoint < waypointCaptureDistance) {
    boolean finishedRoute = evaluateMissionPositionToReach();
    if (finishedRoute)
      positionHoldState = ON;
  }

  crossTrack = crossTrackFactor * crossTrackError;
  groundTrackHeading = constrain(desiredHeading + crossTrack, -MAXCROSSTRACKANGLE, MAXCROSSTRACKANGLE);

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











