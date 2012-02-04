/*
  AeroQuad v3.0 - December 2011
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
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_


/**
 * getAltitudeFromSensors
 *
 * @return the current craft altitude depending of the sensors used
 */
#if defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder) && defined (UseGPS)

  /**
   * @return the most precise altitude, sonar if the reading is ok, otherwise baro merge with GPS
   * it also correct the baro and GPS ground altitude to have a smoot sensor switch
   */
  float getAltitudeFromSensors() {
  
    if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_ALTITUDE) {
      baroGroundAltitude = baroRawAltitude - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  
      gpsGroundAltitude = getGpsAltitude() - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  
      return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]); 
    }
    else {
      return (getBaroAltitude() + getNormalizedGpsAltitude()) / 2;    
    }
  }
  
#elif !defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder) && defined (UseGPS)

  /**
   * @return the most precise altitude, sonar if the reading is ok, otherwise baro merge with GPS
   * it also correct the baro and GPS ground altitude to have a smoot sensor switch
   */
  float getAltitudeFromSensors() {
  
    if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_ALTITUDE) {
      gpsGroundAltitude = getGpsAltitude() - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  
      return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]); 
    }
    else {
      return getNormalizedGpsAltitude();    
    }
  }

#elif defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder) && !defined (UseGPS)

  /**
   * @return the most precise altitude, sonar if the reading is ok, otherwise baro
   * it also correct the baro ground altitude to have a smoot sensor switch
   */
  float getAltitudeFromSensors() {
    
    if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_ALTITUDE) {
      baroGroundAltitude = baroRawAltitude - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  
      return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]); 
    }
    else {
      return getBaroAltitude();    
    }
  }

#elif defined (AltitudeHoldBaro) && !defined (AltitudeHoldRangeFinder) && defined (UseGPS)

  /**
   * @return the the merge of baro and gps altitude
   */
  float getAltitudeFromSensors() {
    return (getBaroAltitude() + getNormalizedGpsAltitude()) / 2;    
  }
  
#elif defined (AltitudeHoldBaro) && !defined (AltitudeHoldRangeFinder) && !defined (UseGPS)

  /**
   * @return the baro altitude
   */
  float getAltitudeFromSensors() {
    return getBaroAltitude();
  }
  
#elif !defined (AltitudeHoldBaro) && !defined (AltitudeHoldRangeFinder) && defined (UseGPS)

  /**
   * @return the the gps altitude
   */
  float getAltitudeFromSensors() {
    return getNormalizedGpsAltitude();    
  }

#elif !defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder) && !defined (UseGPS)

  /**
   * @return the sonar altitude
   */
  float getAltitudeFromSensors() {
    return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
  }

#endif


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
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder || defined (UseGPS)
    if (altitudeHoldState == ON) {
      float currentSensorAltitude = getAltitudeFromSensors();
      if (currentSensorAltitude == INVALID_ALTITUDE) {
        throttle = receiverCommand[THROTTLE];
        return;
      }

      // computer altitude error!
      int altitudeHoldThrottleCorrection = updatePID(altitudeToHoldTarget, currentSensorAltitude, &PID[ALTITUDE_HOLD_PID_IDX]);
      altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      
      // compute throttle z dampening
      int zDampeningThrottleCorrection = -updatePID(0.0, estimatedZVelocity, &PID[ZDAMPENING_PID_IDX]);
      
      if (abs(altitudeHoldThrottle - receiverCommand[THROTTLE]) > altitudeHoldPanicStickMovement) {
        altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
      } else {
        if (receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
          altitudeToHoldTarget += 0.01;
        }
        if (receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
          altitudeToHoldTarget -= 0.01;
        }
      }
      throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection + zDampeningThrottleCorrection;
    }
    else {
      throttle = receiverCommand[THROTTLE];
    }
  #else
    throttle = receiverCommand[THROTTLE];
  #endif
}

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
