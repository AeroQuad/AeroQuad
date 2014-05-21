#ifndef _PLATFORM_FREEFLIGHT_H_
#define _PLATFORM_FREEFLIGHT_H_

#define STM32_BOARD_TYPE "Free Flight"
#define LED_Green  Port2Pin('B', 4)
#define LED_Red    Port2Pin('B', 3)
#define LED_Yellow LED_Red

#include <Device_I2C.h>

#define MPU6000_I2C
#include <Platform_MPU6000.h>
// Gyroscope declaration
#include <Gyroscope_MPU6000.h>
// Accelerometer declaration
#include <Accelerometer_MPU6000.h>

// heading mag hold declaration
#ifdef HeadingMagHold
  #include <Compass.h>
  #define HMC5883L
#endif

// Altitude declaration
#ifdef AltitudeHoldBaro
  #define USE_MS5611_ALTERNATE_ADDRESS
  #define MS5611
#endif

// Battery Monitor declaration
#ifdef BattMonitor
	#define BATT_AREF			3.3		// V

	#define BATT_R_HIGH			10.0	// kOhm
	#define BATT_R_LOW			1.0		// kOhm
	#define BATT_ANALOG_INPUT	Port2Pin('A', 4)
	#define BATT_DIODE_LOSS		0.0
	#define BattDefaultConfig DEFINE_BATTERY(0, BATT_ANALOG_INPUT, (BATT_AREF * (BATT_R_HIGH + BATT_R_LOW) / BATT_R_LOW), BATT_DIODE_LOSS, BM_NOPIN, 0, 0)
#endif

#include <FlightConfigMEGA.h>

/**
 * Put FreeFlight specific initialization need here
 */
void initPlatform() {
  pinMode(LED_Green, OUTPUT);
  for(byte ledloop=0; ledloop<10; ledloop++) {
    digitalWrite(LED_Green, ledloop & 1);
    delay(50);
  }

  pinMode(LED_Red, OUTPUT);
  digitalWrite(LED_Red, LOW);
  pinMode(LED_Yellow, OUTPUT);
  digitalWrite(LED_Yellow, LOW);

  // I2C setup
  Wire.begin(Port2Pin('B', 11), Port2Pin('B', 10)); // I2C1_SDA PB11, I2C1_SCL PB10
  
  switch (flightConfigType) 
  {
    case HEX_Y6 :
    case HEX_PLUS :
    case HEX_X :
      LASTMOTOR = 6;
      break;
    default:
    LASTMOTOR = 4;
  }

}

// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() {
  // Kenny default value, a real accel calibration is strongly recommended
  accelScaleFactor[XAXIS] = 0.0011980000;
  accelScaleFactor[YAXIS] = -0.0012020000;
  accelScaleFactor[ZAXIS] = -0.0011750000;
  #ifdef HeadingMagHold
    magBias[XAXIS]  = 152.000000;
    magBias[YAXIS]  = 24.000000;
    magBias[ZAXIS]  = 16.500000;
  #endif
}


unsigned long previousMeasureCriticalSensorsTime = 0;
unsigned long measureCriticalSensorsTime = 0;
void measureCriticalSensors() {
  // read sensors not faster than every 1 ms
  measureCriticalSensorsTime = micros();
  if ((measureCriticalSensorsTime - previousMeasureCriticalSensorsTime) >= 1000) {
    readMPU6000Sensors();
    measureGyroSum();
    measureAccelSum();
    previousMeasureCriticalSensorsTime = measureCriticalSensorsTime;
  }
}


#endif
