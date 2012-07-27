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

#ifndef _AEROQUAD_HOTT_TELEMETRY_H_
#define _AEROQUAD_HOTT_TELEMETRY_H_

#include "HoTT.h"

static uint8_t minutes = 0;
static uint16_t milliseconds = 0;
static signed int maxAltitude = 500;
static signed int minAltitude = 500;

/* ##################################################################### *
 *                HoTTv4 Common Serial                                   *
 * ##################################################################### */

/**
 * Enables RX and disables TX
 */
static void hottV4EnableReceiverMode() {
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
    UCSR3B &= ~_BV(TXEN3);
    UCSR3B |= _BV(RXEN3);
  #else
    UCSR0B &= ~_BV(TXEN0);
    UCSR0B |= _BV(RXEN0);
  #endif
}

/**
 * Enabels TX and disables RX
 */
static void hottV4EnableTransmitterMode() {
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  
    UCSR3B &= ~_BV(RXEN3);
    UCSR3B |= _BV(TXEN3);
  #else
    UCSR0B &= ~_BV(RXEN0);
    UCSR0B |= _BV(TXEN0); 
  #endif
}

/**
 * Writes out given data to data register.
 */
static void hottV4SerialWrite(uint8_t data) {
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    loop_until_bit_is_set(UCSR3A, UDRE3);
    UDR3 = data;
  #else
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
  #endif
  
  delayMicroseconds(HOTTV4_TX_DELAY);
}

/**
 * Wait until Data Register is empty and
 * TX register is empty.
 */
static void hottV4LoopUntilRegistersReady() {
  delayMicroseconds(HOTTV4_TX_DELAY); 
  
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    loop_until_bit_is_set(UCSR3A, UDRE3);
    loop_until_bit_is_set(UCSR3A, TXC3);
  #else
    loop_until_bit_is_set(UCSR0A, UDRE0);
    loop_until_bit_is_set(UCSR0A, TXC0);
  #endif
}

/**
 * Write out given telemetry data to serial interface.
 * Given CRC is ignored and calculated on the fly.
 */ 
static void hottV4SendBinary(uint8_t *data) {
  uint16_t crc = 0;
  
  /* Enables TX / Disables RX */
  hottV4EnableTransmitterMode();
   
  for (uint8_t index = 0; index < 44; index++) {  
    crc = crc + data[index]; 
    hottV4SerialWrite(data[index]);    
   }
   
  uint8_t crcVal = crc & 0xFF;
  hottV4SerialWrite(crcVal);

  /* Wait until Data Register and TX Register is empty */
  hottV4LoopUntilRegistersReady();
  
  /* Enables RX / Disables TX */
  hottV4EnableReceiverMode();
} 

/* ##################################################################### *
 *                HoTTv4 Module specific Update functions                *
 * ##################################################################### */

/**
 * Updates current direction related on compass information.
 */
static void hottV4UpdateDirection(uint8_t *data) {
	#if defined(HeadingMagHold) || defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
		data[6] = ((int)(trueNorthHeading / M_PI * 180.0) + 360) % 360;
	#else
		data[6] = 0;
	#endif
	}

/**
 * Triggers a notification signal
 * Actually notification is of type HoTTv4Notification, but Wiring lacks in usage
 * of enums, structs, etc. due to their prototyping s*****.
 */
static void hottV4TriggerNotification(uint8_t *data, uint8_t notification) {
  data[2] = notification;
  
  if (notification == HoTTv4NotificationUndervoltage) {
    data[4] = 0x80; // Inverts MikroKopter Telemetry Display for Voltage
  }
}

/**
 * Updates battery voltage telemetry data with given value.
 * Resolution is in 0,1V, e.g. 0x7E == 12,6V.
 * If value is below HOTTV4_VOLTAGE_WARNING, telemetry alarm is triggered
 */
static short hottv4UpdateBattery(uint8_t *data) {
	short voltage = batteryData[0].voltage/10;

    // Activate low voltage alarm
  if (batteryWarning || batteryAlarm) {
    hottV4TriggerNotification(data, HoTTv4NotificationUndervoltage);
  }

  return voltage;
}

static short hottv4UpdateCurrent() {
	if (batteryData[0].cPin != BM_NOPIN) return batteryData[0].current/100;
	return 0;
}

static long hottv4UpdateCapacity() {
	if (batteryData[0].cPin != BM_NOPIN) return batteryData[0].usedCapacity/1000;
	return 0;
}

/**
 * Current relative altitude based on baro or ultrasonic values. 
 * Result is displayed in meter.
 *
 * @param data Pointer to telemetry data frame
 * @param lowByteIndex Index for the low byte that represents the altitude in telemetry data frame
 */
static int32_t hottv4UpdateAlt() {
  int32_t alt = 0;
  
#if defined AltitudeHoldBaro
  alt = (int)getBaroAltitude() + 500;
#endif 

#if defined AltitudeHoldRangeFinder
  if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
	alt = (int)rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] + 500;
  }
#endif
  
  if(alt > maxAltitude) maxAltitude = alt;
  else if(alt < minAltitude) minAltitude = alt;

  return alt;
}


static unsigned int hottv4UpdateAltVario() {
	unsigned int varioSound = 30000;

#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
	if(altitudeHoldState == ON)
	{
		if((receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump))) varioSound = 30100;
		else if((receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump))) varioSound = 29900;
	}
#endif

	return varioSound;
}


/**
 * Updates current flight time by counting the seconds from the moment
 * the copter was armed.
 */
static void hottv4UpdateFlightTime(uint8_t *data) {
  static uint32_t previousEAMUpdate = 0;
  
  uint16_t timeDiff = millis() - previousEAMUpdate;
  previousEAMUpdate += timeDiff;
  
  if (motorArmed) {
    milliseconds += timeDiff;
    
    if (milliseconds >= 60000) {
      milliseconds -= 60000;
      minutes += 1;
    }
  }
  
  data[39] = minutes;
  // Enough accuracy and faster than divide by 1000
  data[40] = (milliseconds >> 10) ;
}

/**
 * Call to initialize HOTTV4
 */
void hottv4Init() {
  hottV4EnableReceiverMode();
    
  #if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    /* Enable PullUps on RX3
     * without signal is to weak to be recognized
     */
    DDRJ &= ~(1 << 0);
    PORTJ |= (1 << 0);
  
    Serial3.begin(19200);
  #endif
}

/* ##################################################################### *
 *                HoTTv4 EAM Module                                      *
 * ##################################################################### */

/**
 * Main method to send EAM telemetry data
 */
static void hottV4SendEAMTelemetry() {  
  uint8_t telemetry_data[] = { 
              0x7C,
              HOTTV4_ELECTRICAL_AIR_MODULE, 
              0x00, /* Alarm */
              HOTTV4_ELECTRICAL_AIR_SENSOR_ID,
              0x00, 0x00, /* Alarm Value 1 and 2 */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Low Voltage Cell 1-7 in 2mV steps */
              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* High Voltage Cell 1-7 in 2mV steps */
              0x00, 0x00, /* Battetry 1 LSB/MSB in 100mv steps, 50 == 5V */
              0x00, 0x00, /* Battetry 2 LSB/MSB in 100mv steps, 50 == 5V */
              0x14, /* Temp 1, Offset of 20. 20 == 0C */ 
              0x14, /* Temp 2, Offset of 20. 20 == 0C */
              0xF4, 0x01, /* Height. Offset -500. 500 == 0 */
              0x00, 0x00, /* Current LSB, MSB 1 = 0.1A */
              0x00, 0x00, /* Drive Voltage */
              0x00, 0x00,  /* mAh */
              0x48, 0x00, /* m2s */ 
              0x78, /* m3s */
              0x00, 0x00, /* RPM. 10er steps, 300 == 3000rpm */
              0x00, /* Electric minutes */
              0x00, /* Electric seconds */
              0x00, /* Speed */
              0x00, /* Version Number */
              0x7D, /* End sign */
              0x00 /* Checksum */
            };
  
  #if defined HOTTV4BATT
    short voltage = hottv4UpdateBattery(telemetry_data);
	telemetry_data[20] = telemetry_data[22] = telemetry_data[30] = voltage;
	telemetry_data[21] = telemetry_data[23] = telemetry_data[31] = (voltage >> 8) & 0xFF;

	short current = hottv4UpdateCurrent();
	telemetry_data[28] = current;
	telemetry_data[29] = (current >> 8) & 0xFF;

	long capacity = hottv4UpdateCapacity();
	telemetry_data[32] = capacity;
	telemetry_data[33] = (capacity >> 8) & 0xFF;
  #endif
  
  #if defined HOTTV4ALTITUDE
    int32_t altitude = hottv4UpdateAlt();
	telemetry_data[26] = altitude;
	telemetry_data[27] = (altitude >> 8) & 0xFF;

	unsigned int varioSound = hottv4UpdateAltVario();
    telemetry_data[11] = telemetry_data[13] = varioSound;
    telemetry_data[12] = telemetry_data[14] = (varioSound >> 8) & 0xFF;
  #endif

  hottv4UpdateFlightTime(telemetry_data);

  // Write out telemetry data as Electric Air Module to serial           
  hottV4SendBinary(telemetry_data);
}

/* ##################################################################### *
 *                HoTTv4 GPS Module                                      *
 * ##################################################################### */

/**
 * Converts unsigned long representation of GPS coordinate back to
 * N Deg MM.SSSS representation and puts it into GPS data frame.
 */
static void updatePosition(uint8_t *data, uint32_t value, uint8_t index) {
  data[index] = (value < 0); 

  uint8_t deg = value / 100000;
  uint32_t sec = (value - (deg * 100000)) * 6;
  uint8_t min = sec / 10000;
  sec = sec % 10000;
  
  uint16_t degMin = (deg * 100) + min;

  data[index+1] = degMin;
  data[index+2] = degMin >> 8; 
  data[index+3] = sec; 
  data[index+4] = sec >> 8;
}

/**
 * Main method to send GPS telemetry data
 */
static void hottV4SendGPSTelemetry() {
  uint8_t telemetry_data[] = { 
              0x7C,
              HOTTV4_GPS_MODULE, 
              0x00, /* Alarm */
              HOTTV4_GPS_SENSOR_ID,
              0x00, 0x00, /* Alarm Value 1 and 2 */
              0x00, /* Flight direction */ 
              0x00, 0x00, /* Velocity */ 
              0x00, 0x00, 0x00, 0x00, 0x00, /* Latitude */
              0x00, 0x00, 0x00, 0x00, 0x00, /* Longitude */
              0x00, 0x00, /* Distance */
              0xF4, 0x01, /* Altitude, 500 = 0m */
              0x78, 0x00, /* m/s, 1 = 0.01m/s */ 
              0x78, /* m/3s, 120 = 0 */
              0x00, /* Number of satelites */ 
              0x00, /* GPS fix character */
              0x00, /* Home direction */
              0x00, /* angle x-direction */
              0x00, /* angle y-direction */
              0x00, /* angle z-direction */
              0x00, 0x00,  /* gyro x */
              0x00, 0x00, /* gyro y */ 
              0x00, 0x00, /* gyro z */
              0x00, /* Vibrations */
              0x00, /* ASCII Free Character 4 */
              0x00, /* ASCII Free Character 5 */
              0x00, /* ASCII Free Character 6 */
              0x00, /* Version Number */
              0x7D, /* End sign */
              0x00 /* Checksum */
            };



#if defined(UseGPS)

  telemetry_data[26] = nbSatelitesInUse;

    if (haveAGpsLock()) {
      updatePosition(telemetry_data, currentPosition.latitude, 9);
      updatePosition(telemetry_data, currentPosition.longitude, 14);

      telemetry_data[27] = telemetry_data[41] = 'f'; // Displays a 'f' for fix

      /** GPS Speed in km/h */
      telemetry_data[7] = getGpsSpeed()*36/1000;

      /** Distance to home */
	  if (navigationState == ON) { 
		    computeDistanceAndBearing(currentPosition, missionPositionToReach);
			telemetry_data[19] = (int)getDistanceMeter();
			telemetry_data[20] = (int)getDistanceMeter() >> 8;
			telemetry_data[39] = HoTTGPSComingHome; // Displays a 'W' for Waypoint
	  }
	  else if(positionHoldState == ON) {
			telemetry_data[19] = 0;
			telemetry_data[20] = 0;
			telemetry_data[39] = HoTTGPSPositionHold; //Displays a 'P' for Position Hold
	  }
	  else {
		  telemetry_data[19] = 0;
		  telemetry_data[20] = 0;
		  telemetry_data[39] = HoTTGPSFree; //Displays a '/' for GPS Mode off 
	  }

	  telemetry_data[28] = (gpsBearing - (int)(trueNorthHeading * RAD2DEG)) * 50;
    }
#endif
          
#if defined HOTTV4ALTITUDE
	int32_t altitude = hottv4UpdateAlt();
	telemetry_data[21] = altitude;
	telemetry_data[22] = (altitude >> 8) & 0xFF;

	unsigned int varioSound = hottv4UpdateAltVario();
	telemetry_data[11] = varioSound;
	telemetry_data[13] = 120;
#endif


  #if defined(HOTTV4DIR) 
    hottV4UpdateDirection(telemetry_data);
  #endif
  
  // Write out telemetry data as GPS Module to serial           
  hottV4SendBinary(telemetry_data);
}

/* ##################################################################### *
 *                HoTTv4 Vario Module                                    *
 * ##################################################################### */

/**
 * Main method to send Vario telemetry data
 */
static void hottV4SendVarioTelemetry() {
  uint8_t telemetry_data[] = { 
              0x7C,
              HOTTV4_VARIO_MODULE, 
              0x00, /* Alarm */
              HOTTV4_VARIO_SENSOR_ID,
              0x00, /* Inverse status */
              0xF4, 0x01, /* Current altitude */ 
              0xF4, 0x01, /* Max. altitude */ 
              0xF4, 0x01, /* Min. altitude */
              0x30, 0x75, /* m/s */
              0x30, 0x75, /* m/3s  */
              0x30, 0x75, /* m/10s */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* ASCII */
              0x00,                   /* ASCII */
              0x00, 0x00, 0x00, 0x00, /* free */
              0x00, /* Version Number */
              0x7D, /* End sign */
              0x00  /* Checksum */
            };

#if defined HOTTV4ALTITUDE
  int32_t altitude = hottv4UpdateAlt();
  telemetry_data[5] = altitude;
  telemetry_data[6] = (altitude >> 8) & 0xFF;

  telemetry_data[7] = maxAltitude;
  telemetry_data[9] = minAltitude;

  unsigned int varioSound = hottv4UpdateAltVario();
  telemetry_data[11] = telemetry_data[13] = telemetry_data[15] = varioSound;
  telemetry_data[12] = telemetry_data[14] = telemetry_data[16] = (varioSound >> 8) & 0xFF;

  if(altitudeHoldState == ON) {
	  telemetry_data[38] = '.';
	  }
  else if (altitudeHoldState == ALTPANIC) {
	  telemetry_data[38] = '!';
	  }
#endif

  // Buffer for the available 21 ASCII + \0 chars
  char text[VARIO_ASCIIS+1];
  
  if(flightMode == ATTITUDE_FLIGHT_MODE) snprintf(text, VARIO_ASCIIS+1, HOTTV4_VARIO_ATTITUDE);
  else snprintf(text, VARIO_ASCIIS+1, HOTTV4_VARIO_RATE);

  uint8_t offset = (VARIO_ASCIIS - strlen(text)) / 2;

  for(uint8_t index = 0; (index + offset) < VARIO_ASCIIS; index++) {
    if (text[index] != 0x0) {
      // 17 == start byte for ASCII
      telemetry_data[17+index+offset] = text[index];
    } else {
      break;
    }
  }  
            
  // Write out telemetry data as Vario Module to serial           
  hottV4SendBinary(telemetry_data);
}

/* ##################################################################### *
 *                HoTTv4 Text Mode                                       *
 * ##################################################################### */

/**
 * Check if enough time has been elapsed since last telemetry update to
 * prevent too much interference with motor control.
 */
uint8_t canSendTelemetry() {
  static uint16_t lastTimeUpdated = 0;
  
  if ((millis() - lastTimeUpdated) > HOTTV4_UPDATE_INTERVAL) {
    lastTimeUpdated = millis();
    
    return 1;
  } else {
    return 0;
  }
}

/**
 * Main entry point for HoTTv4 telemetry
 */
uint8_t hottV4Hook(uint8_t serialData) {
  switch (serialData) {
    case HOTTV4_GPS_MODULE:
      if (canSendTelemetry()) {
        hottV4SendGPSTelemetry();
      }
      break;
    
    case HOTTV4_ELECTRICAL_AIR_MODULE:
      if (canSendTelemetry()) {
        hottV4SendEAMTelemetry();
      }
      break;
         
    case HOTTV4_VARIO_MODULE:
      if (canSendTelemetry()) {
        hottV4SendVarioTelemetry();
      }
      break;

    default:
      return serialData;
  }

  return 0;
}

#endif 