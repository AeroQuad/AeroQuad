/*
  AeroQuad v3.0 - May 2011
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

/*
 * eventually, it's about the normal DCM processor and I KNOW that this is heavy!
 * Still, this is the best result I did get with my knowledge that give good attitude
 * estimator with the AGR and pretty good true heading computation at the same time
 * 
 * @Kenny9999
 * I'm open to anything more lighweight working and FLIGHT TESTED
 */
#ifndef _AQ_HEADING_FUSION_PROCESSOR_COMP_FILTER_
#define _AQ_HEADING_FUSION_PROCESSOR_COMP_FILTER_

#include "Compass.h"

#if defined UseGPS
  #include "MagnetometerDeclinationDB.h"
#endif  

float trueNorthHeading = 0.0;
float compassDeclination = 0.0;

float filter1 = 1.0 / (1.0 + 0.002);
float filter2 = 1 - filter1;
int headingGyroZero = 0;
float localHeading = 0.0;

void initializeHeadingFusion() {

  headingGyroZero = gyroZero[ZAXIS];
  gyroHeading = getAbsoluteHeading();
}

void calculateHeading() {

  float compass = getAbsoluteHeading();
  if (compass > PI) {
    compass = fmod(compass,PI) - PI;
  }

  if (headingGyroZero != gyroZero[ZAXIS]) {
    gyroHeading = localHeading;
    headingGyroZero = gyroZero[ZAXIS];
  }

  float adjustedGyroHeading = fmod(gyroHeading,(PI*2));
  int divider = abs(adjustedGyroHeading) / PI;
  if (adjustedGyroHeading > 0) {
    if (divider == 1) {
	  adjustedGyroHeading = -(PI - fmod(adjustedGyroHeading,PI));
    }
  }
  else {
    if (divider == 1) {
	  adjustedGyroHeading = fmod(adjustedGyroHeading,PI) + PI;
    }
  }

  // Complementry filter from http://chiefdelphi.com/media/papers/2010
  localHeading = (filter1 * adjustedGyroHeading) + (filter2 * getAbsoluteHeading());

  trueNorthHeading = localHeading;
  #if defined UseGPS
    if( compassDeclination != 0.0 ) {

	  trueNorthHeading = trueNorthHeading + compassDeclination;
	  if (trueNorthHeading > M_PI)  {  // Angle normalization (-180 deg, 180 deg)
	    trueNorthHeading -= (2.0 * M_PI);
	  } 
	  else if (trueNorthHeading < -M_PI){
	    trueNorthHeading += (2.0 * M_PI);
	  }
    }
  #endif
}

#if defined UseGPS
  void setDeclinationLocation(long lat, long lon) {
    // get declination ( in radians )
    compassDeclination = getMagnetometerDeclination(lat, lon);    
  }
#endif  

#endif