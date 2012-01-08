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

/**
 * Altitude control processor do the premilary treatment on throttle correction
 * to control the altitude of the craft. It then modify directly the 
 * throttle variable use by the motor matrix calculation
 */

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

//void computeEstimatedAltitude(float currentSensorAltitude) {
//
//  float altitudeError = currentSensorAltitude - estimatedAltitude;
//  altitudeIntegratedError += altitudeError;
//  altitudeIntegratedError = constrain(altitudeIntegratedError,-0.5,0.5);
//  
//  // Gravity vector correction and projection to the local Z
//  float zVelocity = ((filteredAccelYaw * (1 - accelOneG * invSqrt(isq(filteredAccelRoll) + isq(filteredAccelPitch) + isq(filteredAccelYaw)))) / 10) + altitudeIntegratedError;
//
//  float altitudeDelta = (zVelocity * G_Dt) + (altitudeError * G_Dt);
//  estimatedAltitude = ((estimatedZVelocity + altitudeDelta) * G_Dt) +  (altitudeError * G_Dt);
//  estimatedZVelocity += zVelocity;
//}




/**
 * getAltitudeFromSensors
 *
 * @return the current craft altitude depending of the sensors used
 */
#if defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder)

  /**
   * @return the most precise altitude, sonar if the reading is ok, otherwise baro
   * it also correct the baro ground altitude to have a smoot sensor switch
   */
  float getAltitudeFromSensors() {
    
    if (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] != INVALID_ALTITUDE) {
      baroGroundAltitude = oversampledBaroAltitude - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];  
      return (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]); 
    }
    else {
      return oversampledBaroAltitude;    
    }
  }
  
#elif defined (AltitudeHoldBaro) && !defined (AltitudeHoldRangeFinder)

  /**
   * @return the baro altitude
   */
  float getAltitudeFromSensors() {
    return oversampledBaroAltitude;
  }
  
#elif !defined (AltitudeHoldBaro) && defined (AltitudeHoldRangeFinder)
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
  #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
    if (altitudeHoldState == ON) {
      float currentSensorAltitude = getAltitudeFromSensors();
      if (currentSensorAltitude == INVALID_ALTITUDE) {
        throttle = receiverCommand[THROTTLE];
        return;
      }

      int altitudeHoldThrottleCorrection = updatePID(altitudeToHoldTarget, currentSensorAltitude, &PID[ALTITUDE_HOLD_PID_IDX]);
      altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      if (altitudeHoldThrottleCorrection > 0) {
        altitudeHoldThrottleCorrection / 2;  // going down 2 time slower than when we climb!
      }
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
      throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection;
    }
    else {
      throttle = receiverCommand[THROTTLE];
    }
  #else
    throttle = receiverCommand[THROTTLE];
  #endif
}



#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_

