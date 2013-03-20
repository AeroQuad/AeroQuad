/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_BAROMETRIC_SENSOR_
#define _AQ_BAROMETRIC_SENSOR_

#include "Arduino.h"
#include "GlobalDefined.h"

float baroAltitude      = 0.0; 
float baroRawAltitude   = 0.0;
float baroGroundAltitude = 0.0;
float baroSmoothFactor   = 0.02;
float lastbaroAltitude = 0.0;
  
// **********************************************************************
// The following function calls must be defined inside any new subclasses
// **********************************************************************
void initializeBaro(); 
void measureBaro();
void measureBaroSum();
void evaluateBaroAltitude();
  
// *********************************************************
// The following functions are common between all subclasses
// *********************************************************
const float getBaroAltitude() {
  return baroAltitude - baroGroundAltitude;
}
 
void measureGroundBaro() {
  // measure initial ground pressure (multiple samples)
  float altSum = 0.0;
  for (int i=0; i < 25; i++) {
    measureBaro();
	altSum += baroRawAltitude;
    delay(12);
  }
  baroGroundAltitude = altSum / 25;
}
const float getdeltaAltitude() {
  return baroAltitude - lastbaroAltitude;						// using filtered data (baroAltitude and lastbaroAltitude)
}

/**********************************************************
 ********************** digitalSmooth *********************
 **********************************************************/

float digitalSmooth(float rawIn, float *sensSmoothArray){				// "float *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  static float total = 0.0;								// total values after all processing
  static int i, k = 0;									// loop variables
  static boolean done = false;								// used to find first total

  if (!done) {										// FIND FIRST TOTAL
	for (k=0; k < filterSamples; k++){
		total += sensSmoothArray[k];
	}
	done = true;
  }

  i = (i + 1) % filterSamples;								// increment counter and roll over if necc.
											// % (modulo operator) rolls over variable
  total -= sensSmoothArray[i];								// drop last valve from total
  sensSmoothArray[i] = rawIn;								// input new data into the oldest slot
  total += rawIn;									// add new value to total
  
  return total / filterSamples;
}

/**********************************************************
 *************** Determine vertical rate (+/-) ************
 **********************************************************/
// called in 50 Hz slice

float deltaAltitudeRateFeet( float time_increment ) {	 				// returns feet per second
  static float smoothArray[filterSamples];						// array for holding smoothed values for New Altitude 

  #define meters_to_feet	3.28084							// convert to feet per second to call it something
											// were not planning on displaying a number so matters not
  float delta_Factor = meters_to_feet * time_increment;					// called in 50 Hz slice

  float climb_fallRate = ( baroAltitude - lastbaroAltitude ) * delta_Factor;		

  climb_fallRate = digitalSmooth(climb_fallRate, smoothArray);				// so our eyes don't vibrate out of our skull

  return climb_fallRate;								// return smoothed, despiked climbFallRate
}

#endif
