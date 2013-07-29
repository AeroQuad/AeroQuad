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
      setProjectionLocation(currentPosition); // Do we need this anymore?
      
      #if defined UseGPSNavigator
        isRouteInitialized = false;
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

long pwm(float angle) {
  // 0 to 90 degrees = 0 to 1500 PWM
  return (long)angle*16.6666667;
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

void estimateVelocity(double *velVector, long gpsSpeed, long gpsBearing) {
  // Currently this function will estimate velocity with GPS info only
  // Future version needs to calculate velocity from accelerometers also
  // velocity vector units are in cm/s
  double speed = (double)gpsSpeed/100000.0;
  double heading = (double)gpsBearing * GPS2RAD;
  if (heading > PI)
    heading -= 2*PI;

  velVector[0] = speed*cos(heading);
  velVector[1] = speed*sin(heading);
  velVector[2] = 0.0; // figure out how to estimate velocity in altitude
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
bool updateWaypoints() { // returns false if next waypoint available, true if at end of route

  if (waypointIndex == -1) { // if mission have not been started
    waypointIndex++;
  }

  if ((waypointIndex < MAX_WAYPOINTS) && (distanceToNextWaypoint < waypointCaptureDistance)) {
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
  }
}

void loadNewRoute() {
  waypointIndex = -1;
  updateWaypoints();
}

/**
 * Process navigation
 */

void processNavigation() {
  #define MAXBANKANGLE 10 // (degrees)
  #define MAXCROSSTRACKANGLE 90 // (degrees)
  #define MAXCROSSTRACKDISTANCE 15 // (meters)
  const double crossTrackFactor = -MAXCROSSTRACKANGLE/MAXCROSSTRACKDISTANCE;

  // Convert lat/lon to ECI unit vector
  positionVector(presentPosition, currentPosition);

  // Calculate track angle error
  vectorCrossProductDbl(presentPositionEast, zVector, presentPosition);
  vectorNormalize(presentPositionEast);
  vectorCrossProductDbl(presentPositionNorth, presentPosition, presentPositionEast);
  vectorNormalize(presentPositionNorth);
  desiredHeading = deg(atan2(vectorDotProductDbl(normalVector, presentPositionNorth), vectorDotProductDbl(negNormalVector, presentPositionEast)));
  currentHeading = adjustHeading(heading, desiredHeading);
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
  distanceToGoAlongPath = earthRadius * acos(vectorDotProductDbl(toVector, alongPathVector));
  distanceToGoPosition = earthRadius * acos(vectorDotProductDbl(toVector, presentPosition));

  if (distanceToNextWaypoint < waypointCaptureDistance) {
    bool routeisFinished = updateWaypoints();
    if (routeisFinished)
      positionHoldState = ON;
  }

  crossTrack = constrain(crossTrackFactor * crossTrackError, -MAXCROSSTRACKANGLE, MAXCROSSTRACKANGLE);
  groundTrackHeading = desiredHeading + crossTrack; // TODO: update to fix issue around +/-180

  gpsRollAxisCorrection = rad(constrain(groundTrackHeading-currentHeading, -MAXBANKANGLE, MAXBANKANGLE))/PWM2RAD;
  gpsPitchAxisCorrection = 0;
  gpsYawAxisCorrection = rad(constrain(groundTrackHeading-currentHeading, -MAXBANKANGLE, MAXBANKANGLE))/PWM2RAD;
}

void processPositionHold()
{
  const long maxPosAngle = pwm(10.0); // Calculate desired degrees in PWM
  latDelta = positionHoldPointToReach.latitude - currentPosition.latitude;
  lonDelta = positionHoldPointToReach.longitude - currentPosition.longitude;

  posRollCommand = -updatePID(0, lonDelta*cos(trueNorthHeading) - latDelta*sin(trueNorthHeading), &PID[GPSROLL_PID_IDX]);
  gpsRollAxisCorrection = constrain(posRollCommand, -maxPosAngle, maxPosAngle);

  posPitchCommand = -updatePID(0, lonDelta*sin(trueNorthHeading) + latDelta*cos(trueNorthHeading), &PID[GPSPITCH_PID_IDX]);
  gpsPitchAxisCorrection = constrain(posPitchCommand, -maxPosAngle, maxPosAngle);

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











