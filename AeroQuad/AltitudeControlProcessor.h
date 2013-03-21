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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_


#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

#define INVALID_THROTTLE_CORRECTION -1000
#define ALTITUDE_BUMP_SPEED 0.01

<<<<<<< HEAD
#define numberofSamplestoFilter 23			// filterSamples should  be an odd number, no smaller than 3									// filterSamples should  be an odd number, no smaller than 3
float lastbaroAltitude = 0.0;


=======
>>>>>>> 7ceb250e89430a92431002856b9890e5ffba49c3
/**
 * processAltitudeHold
 * 
 * This function is responsible to process the throttle correction 
 * to keep the current altitude if selected by the user 
 */
void processAltitudeHold()
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325

  if (altitudeHoldState == ON) {
    int altitudeHoldThrottleCorrection = INVALID_THROTTLE_CORRECTION;
    // computer altitude error!
    #if defined AltitudeHoldRangeFinder
      if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
        if (sonarAltitudeToHoldTarget == INVALID_RANGE) {
          sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
        }
        altitudeHoldThrottleCorrection = updatePID(sonarAltitudeToHoldTarget, rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX], &PID[SONAR_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      }
    #endif
    #if defined AltitudeHoldBaro
      if (altitudeHoldThrottleCorrection == INVALID_THROTTLE_CORRECTION) {
        altitudeHoldThrottleCorrection = updatePID(baroAltitudeToHoldTarget, getBaroAltitude(), &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      }
    #endif        
    if (altitudeHoldThrottleCorrection == INVALID_THROTTLE_CORRECTION) {
      throttle = receiverCommand[THROTTLE];
      return;
    }
    
    // ZDAMPENING COMPUTATIONS
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      float zDampeningThrottleCorrection = -updatePID(0.0, estimatedZVelocity, &PID[ZDAMPENING_PID_IDX]);
      zDampeningThrottleCorrection = constrain(zDampeningThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
    #endif

    
    if (abs(altitudeHoldThrottle - receiverCommand[THROTTLE]) > altitudeHoldPanicStickMovement) {
      altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } 
    else {
      
      if (receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        #if defined AltitudeHoldBaro
          baroAltitudeToHoldTarget += ALTITUDE_BUMP_SPEED;
        #endif
        #if defined AltitudeHoldRangeFinder
          float newalt = sonarAltitudeToHoldTarget + ALTITUDE_BUMP_SPEED;
          if (isOnRangerRange(newalt)) {
            sonarAltitudeToHoldTarget = newalt;
          }
        #endif
      }
      
      if (receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        #if defined AltitudeHoldBaro
          baroAltitudeToHoldTarget -= ALTITUDE_BUMP_SPEED;
        #endif
        #if defined AltitudeHoldRangeFinder
          float newalt = sonarAltitudeToHoldTarget - ALTITUDE_BUMP_SPEED;
          if (isOnRangerRange(newalt)) {
            sonarAltitudeToHoldTarget = newalt;
          }
        #endif
      }
    }
    throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection + zDampeningThrottleCorrection;
  }
  else {
    throttle = receiverCommand[THROTTLE];
  }
  
  
  // compute baro velocity rate
  #if defined(AltitudeHoldBaro)
    climbFallRate = deltaAltitudeRateFeet(50.0);   		// update altitude rate in feet per second
  #endif
}

<<<<<<< HEAD
=======



#if defined(AltitudeHoldBaro)

#define metersToFeet	3.28084                                 // convert to feet per second to call it something
#define filterSamples 23  	                                // filterSamples should  be an odd number, no smaller than 3
float lastbaroAltitude = 0.0;


const float getDeltaAltitude() {
  return baroAltitude - lastbaroAltitude;  			// using filtered data (baroAltitude and lastbaroAltitude)
}

>>>>>>> 7ceb250e89430a92431002856b9890e5ffba49c3
/**********************************************************
 ********************** digitalSmooth *********************
 **********************************************************/

<<<<<<< HEAD
float digitalSmooth(float rawIn, float *baroSmoothArray){			// some storage for holding out barometer samples
  static float sumSamplestoFilter = 0.0;					// total samples after all processing
  static int thisBaroSample, currentSampleSlot = 0;				// loop variables
  static boolean done = false;							// used to find totalfilterSamples

  if (!done) {																// find total samples while storing
	for (currentSampleSlot=0; currentSampleSlot < numberofSamplestoFilter; currentSampleSlot++){
		sumSamplestoFilter += baroSmoothArray[currentSampleSlot];
	}
	done = true;
=======
float digitalSmooth(float rawIn, float *sensSmoothArray) {     	// "float *sensSmoothArray" passes an array to the function - the asterisk indicates the array name is a pointer
  static float total = 0.0;					// total values after all processing
  static int i, k = 0;						// loop variables
  static boolean done = false;					// used to find first total

  if (!done) {							// FIND FIRST TOTAL
    for (k=0; k < filterSamples; k++) {
      total += sensSmoothArray[k];
    }
    done = true;
>>>>>>> 7ceb250e89430a92431002856b9890e5ffba49c3
  }

  thisBaroSample = (thisBaroSample + 1) % numberofSamplestoFilter;		// increment counter and roll over when needed
										// % (modulo operator) rolls over variable
  sumSamplestoFilter -= baroSmoothArray[thisBaroSample];			// drop last value from total
  baroSmoothArray[thisBaroSample] = rawIn;					// input new data into the oldest slot
  sumSamplestoFilter += rawIn;							// add new value to total
  
  return sumSamplestoFilter/numberofSamplestoFilter;
}

/**********************************************************
 *************** Determine vertical rate (+/-) ************
 **********************************************************/
<<<<<<< HEAD
// this routine must be called in 50 Hz slice

float deltaAltitudeRateMeters( float time_increment ) {	 			// returns meters per second
  static float smoothArray[numberofSamplestoFilter];				// array for holding smoothed values for New Altitude 

  float climbFallRate = (baroAltitude-lastbaroAltitude)*time_increment;		// called in 50 Hz slice

  climbFallRate = digitalSmooth(climbFallRate, smoothArray);			// so our eyes don't vibrate out of our skull
  lastbaroAltitude = baroAltitude;

  return climbFallRate;								// return smoothed, despiked climbFallRate
=======
void deltaAltitudeRateFeet( float time_increment ) {	        // returns feet per second
  static float smoothArray[filterSamples];                      // array for holding smoothed values for New Altitude 
	                                                        // were not planning on displaying a number so matters not
  float delta_Factor = metersToFeet * time_increment;     	// called in 50 Hz slice
  float climbFallRate = ( baroAltitude - lastbaroAltitude ) * delta_Factor;		
  climbFallRate = digitalSmooth(climbFallRate, smoothArray);	// so our eyes don't vibrate out of our skull
  lastbaroAltitude = baroAltitude;
>>>>>>> 7ceb250e89430a92431002856b9890e5ffba49c3
}

#endif

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
