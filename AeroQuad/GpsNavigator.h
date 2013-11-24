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

double calculateGPSDistance(GeodeticPosition waypoint1, GeodeticPosition waypoint2) {
  double lat1 = (double)waypoint1.latitude * GPS2RAD;
  double lon1 = (double)waypoint1.longitude * GPS2RAD;
  double lat2 = (double)waypoint2.latitude * GPS2RAD;
  double lon2 = (double)waypoint2.longitude * GPS2RAD;
  return acos(sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2) * cos(lon2-lon1)) * earthRadius;
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  return acos(sin(lat1)*sin(lat2) + cos(lat1)*cos(lat2) * cos(lon2-lon1)) * earthRadius;
}

double calculateCourse(double lat1, double lon1, double lat2, double lon2) {
  // from http://www.movable-type.co.uk/scripts/latlong.html
  // assume all units are radians
  double y = sin(lon2-lon1) * cos(lat2);
  double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(lon2-lon1);
  return atan2(y, x);
}

void estimateGPSVelocity() {
  // This function is current placed in the 1Hz task to provide m/s velocity estimation
  // 1 Hz updates were chosen because uBlox6 GPS is setup for 1Hz update rate
  double currentLat = (double)currentPosition.latitude * GPS2RAD;
  double currentLon = (double)currentPosition.longitude * GPS2RAD;
  double deltaDist = calculateDistance(previousLat, previousLon, currentLat, currentLon);
  estSpeed = deltaDist/(double)G_Dt*100.0; // units in cm/s
  estCourse = deg(calculateCourse(previousLat, previousLon, currentLat, currentLon));
  previousLat = currentLat;
  previousLon = currentLon;
}

void estimateAccVelocity() {
  const double smoothFactor = 1.0;
  const double deltaT = 0.010;

  // estimate X, Y, Z velocities
  for (int axis=XAXIS; axis<=ZAXIS; axis++) {
    smoothedAcc[axis] +=  acc[axis]*deltaT*9.80665; //filterSmooth(acc[axis] * deltaT * 9806.65, smoothedAcc[axis], smoothFactor); // cm/s
  }

  accVelocity[0] = smoothedAcc[XAXIS]*cos(trueNorthHeading) - smoothedAcc[YAXIS]*sin(trueNorthHeading);
  accVelocity[1] = smoothedAcc[XAXIS]*sin(trueNorthHeading) + smoothedAcc[YAXIS]*cos(trueNorthHeading);
  accVelocity[2] = 0.0;
}

void estimateVelocity() {
  const double smoothFactor = 1.0;
  const double blendFactor = 0.3;

  double gpsCourse = (double)gpsData.course/10.0E2;
  if (gpsCourse>180.0) gpsCourse -= 360.0;
  gpsCourse = rad(gpsCourse);
  gpsVelocity[0] = (double)getGpsSpeed()*cos(gpsCourse); //Vnorth
  gpsVelocity[1] = (double)getGpsSpeed()*sin(gpsCourse); //Veast
  gpsVelocity[2] = 0.0; // figure out how to estimate velocity in altitude later

  // estimate X, Y, Z velocities
  for (int axis=XAXIS; axis<=ZAXIS; axis++) {
    smoothedAcc[axis] =  filterSmooth(acc[axis] * G_Dt * 9806.65, smoothedAcc[axis], smoothFactor); // cm/s
  }

  accVelocity[0] = smoothedAcc[XAXIS]*cos(trueNorthHeading) - smoothedAcc[YAXIS]*sin(trueNorthHeading);
  accVelocity[1] = smoothedAcc[XAXIS]*sin(trueNorthHeading) + smoothedAcc[YAXIS]*cos(trueNorthHeading);
  accVelocity[2] = 0.0;

  for (int axis=XAXIS; axis <=ZAXIS; axis++) {
    if (getGpsSpeed() <= 1)
      velocityVector[axis] = 0.0;
    else
      velocityVector[axis] = filterSmooth(accVelocity[axis], gpsVelocity[axis], blendFactor);
  }
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
  if (waypointIndex == -2) { // creates one time path from current position to first waypoint
    missionPositionToReach.latitude = currentPosition.latitude;
    missionPositionToReach.longitude = currentPosition.longitude;
    missionPositionToReach.altitude = currentPosition.altitude;
    fromWaypoint = currentPosition;
    toWaypoint = waypoint[0];
    followingWaypoint = waypoint[1];
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
    waypointIndex = -1;
    return false;
  }

  if ((waypointIndex < MAX_WAYPOINTS)) {
    waypointIndex++;
  }

  if (waypointIndex >= MAX_WAYPOINTS || waypoint[waypointIndex].altitude == GPS_INVALID_ALTITUDE) { // if mission is completed, last step is to go home 2147483647 == invalid altitude
    missionPositionToReach.latitude = homePosition.latitude;
    missionPositionToReach.longitude = homePosition.longitude;
    missionPositionToReach.altitude = homePosition.altitude;
    return true; // finished route
  }
  else {
    missionPositionToReach.latitude = waypoint[waypointIndex].latitude;
    missionPositionToReach.longitude = waypoint[waypointIndex].longitude;
    missionPositionToReach.altitude = waypoint[waypointIndex].altitude;
    fromWaypoint = waypoint[waypointIndex];
    toWaypoint = waypoint[waypointIndex+1];
    followingWaypoint = waypoint[waypointIndex+2];
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
  waypointIndex = -2; // create path from current position to first waypoint
  updateWaypoints();
}

/**
 * Process navigation
 */

void processNavigation() {
  #define MAXBANKANGLE 30 // degrees, should I change all angle errors to max bank angle error?

  #define MAXPWM 500 // max PWM to output
  #define MAXTRACKANGLE 60 // degrees
  const double Deg2PWMFactor = MAXPWM/MAXTRACKANGLE; // convert degrees into PWM

  #define MAXCROSSTRACKANGLE 60 // degrees
  #define MAXCROSSTRACKDISTANCE 15 // meters
  const double Meters2DegFactor = MAXCROSSTRACKANGLE/MAXCROSSTRACKDISTANCE; // convert meters into degrees

  // Convert lat/lon to ECI unit vector
  positionVector(presentPosition, currentPosition);

  // Calculate track angle error
  vectorCrossProductDbl(presentPositionEast, zVector, presentPosition);
  vectorNormalize(presentPositionEast);
  vectorCrossProductDbl(presentPositionNorth, presentPosition, presentPositionEast);
  vectorNormalize(presentPositionNorth);
  desiredHeading = deg(atan2(vectorDotProductDbl(normalVector, presentPositionNorth), vectorDotProductDbl(negNormalVector, presentPositionEast)));
  currentHeading = adjustHeading(heading, desiredHeading);
  // units of track angle error is degrees
  trackAngleError = constrain(desiredHeading-currentHeading, -MAXTRACKANGLE, MAXTRACKANGLE);

  // Calculate cross track error
  vectorCrossProductDbl(normalPerpendicularVector, presentPosition, normalVector);
  vectorNormalize(normalPerpendicularVector);
  vectorCrossProductDbl(alongPathVector, normalVector, normalPerpendicularVector);
  vectorNormalize(alongPathVector);
  crossTrack = earthRadius * atan2(vectorDotProductDbl(negNormalVector, presentPosition), vectorDotProductDbl(alongPathVector, presentPosition));
  // units of cross track error is converted to degrees
  crossTrackError = -constrain(crossTrack*Meters2DegFactor, -MAXCROSSTRACKANGLE, MAXCROSSTRACKANGLE);

  // Calculate distance to next waypoint
  vectorCrossProductDbl(normalRangeVector, presentPosition, toVector);
  vectorNormalize(normalRangeVector);
  vectorCrossProductDbl(rangeVector, toVector, normalRangeVector);
  vectorNormalize(rangeVector);
  distanceToNextWaypoint = earthRadius * atan2(vectorDotProductDbl(rangeVector, presentPosition), vectorDotProductDbl(presentPosition, toVector));
  distanceToGoAlongPath = earthRadius * acos(vectorDotProductDbl(toVector, alongPathVector));
  distanceToGoPosition = earthRadius * acos(vectorDotProductDbl(toVector, presentPosition));
  distanceToFollowingWaypoint = calculateGPSDistance(currentPosition, followingWaypoint);
  testDistanceWaypoint = calculateGPSDistance(currentPosition, toWaypoint);

  // These corrections need to be PWM centered around 0
  gpsPitchAxisCorrection = forwardSpeed * Deg2PWMFactor * 2.5; // pitch forward in degrees converted to radians
  gpsRollAxisCorrection = constrain((trackAngleError+crossTrackError), -MAXBANKANGLE, MAXBANKANGLE) * Deg2PWMFactor;
  gpsYawAxisCorrection = constrain((trackAngleError+crossTrackError), -MAXBANKANGLE, MAXBANKANGLE) * Deg2PWMFactor;

  if ((distanceToNextWaypoint < waypointCaptureDistance) || (distanceToFollowingWaypoint < distanceToNextWaypoint)) {
    bool routeisFinished = updateWaypoints();
    if (routeisFinished) {
      positionHoldState = ON;
      navigationState = OFF;
      gpsPitchAxisCorrection = 0.0;
    }
  }
}

void processPositionHold()
{
  const long maxPosAngle = pwm(10.0); // Calculate desired degrees in PWM
  latDelta = positionHoldPointToReach.latitude - currentPosition.latitude;
  lonDelta = positionHoldPointToReach.longitude - currentPosition.longitude;

  posRollCommand = -updatePID(0, lonDelta*cos(trueNorthHeading) - latDelta*sin(trueNorthHeading), &PID[GPSROLL_PID_IDX]);
  gpsRollAxisCorrection = 0.0; //constrain(posRollCommand, -maxPosAngle, maxPosAngle);

  posPitchCommand = -updatePID(0, lonDelta*sin(trueNorthHeading) + latDelta*cos(trueNorthHeading), &PID[GPSPITCH_PID_IDX]);
  gpsPitchAxisCorrection = 0.0; //constrain(posPitchCommand, -maxPosAngle, maxPosAngle);

  gpsYawAxisCorrection = 0;

  PIDdata velPID; // TODO: add to EEPROM
  velPID.P = 1.0;
  velPID.I = 0.0;
  velPID.D = 0.0;
  //estimateVelocity(velocityVector, gpsData.speed, gpsData.course);
  //velRollCommand = updatePID(0, velocityVector[YAXIS], &velPID); // cm/s
  //velPitchCommand = updatePID(0, velocityVector[XAXIS], &velPID); // cm/s
  velRollCommand = updatePID(0, smoothedAcc[YAXIS], &PID[GPSROLL_PID_IDX]);
  velPitchCommand = updatePID(0, smoothedAcc[XAXIS], &PID[GPSPITCH_PID_IDX]);
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











