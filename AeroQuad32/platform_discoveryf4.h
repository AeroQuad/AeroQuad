#ifndef _PLATFORM_DISCOVERYF4_H_
	
#define _PLATFORM_DISCOVERYF4_H_

static byte stm32_motor_mapping[] = {
  Port2Pin('C',  9),
  Port2Pin('C',  8),
  Port2Pin('C',  7),
  Port2Pin('C',  6),
  // pin mapping for motor 5-8 not specified, yet
  Port2Pin('A', 15),
  Port2Pin('B',  3),
  Port2Pin('B',  4),
  Port2Pin('B',  5)
};

#ifdef RECEIVER_STM32PPM
  static byte receiverPinPPM = Port2Pin('E', 9);
#else
  static byte receiverPin[] = {
	Port2Pin('E',  9),
	Port2Pin('E', 11),
	Port2Pin('E', 13),
	Port2Pin('E', 14),
	Port2Pin('B',  4),
	Port2Pin('B',  5),
	Port2Pin('B',  0),
	Port2Pin('B',  1)
  };
#endif

#define STM32_BOARD_TYPE "Discovery F4"
#define LED_Green  Port2Pin('D', 12)
#define LED_Red    Port2Pin('D', 14)
#define LED_Yellow Port2Pin('D', 13)

#include <Device_I2C.h>
#include "log.h"
#include "log.c"

//#include <Gyroscope_ITG3200.h>
#include <Gyroscope_MPU6000.h>

//#include <Accelerometer_BMA180.h>
#include <Accelerometer_MPU6000.h>

// heading mag hold declaration
#ifdef HeadingMagHold
  #define HMC5883L
#endif

// Altitude declaration
#ifdef AltitudeHoldBaro
  //#define BMP085
  #define MS5611
#endif

// Battery Monitor declaration
#ifdef BattMonitor
  #define BATT_AREF			    3.3		// V

  #define BATT_R_HIGH       10.0	// kOhm
  #define BATT_R_LOW        1.5		// kOhm
  #define BATT_ANALOG_INPUT	Port2Pin('C', 0)
  #define BATT_DIODE_LOSS		0.0
  #define BattDefaultConfig DEFINE_BATTERY(0, BATT_ANALOG_INPUT, (BATT_AREF * (BATT_R_HIGH + BATT_R_LOW) / BATT_R_LOW), BATT_DIODE_LOSS, BM_NOPIN, 0, 0)
#endif


void HardCodedAxisCalibration()
{
  initSensorsZeroFromEEPROM();
  if((accelScaleFactor[XAXIS] == 1.0 && accelScaleFactor[YAXIS] == 1.0 && accelScaleFactor[ZAXIS] == 1.0) ||
     (accelScaleFactor[XAXIS] == 0.0 && accelScaleFactor[YAXIS] == 0.0 && accelScaleFactor[ZAXIS] == 0.0)) {
    #ifdef _AEROQUAD_PLATFORM_MPU6000_H_
	  accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = 0.00119;
    #else
	  accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = 0.0048;
    #endif
    accelScaleFactor[YAXIS] *= -1;
    accelScaleFactor[ZAXIS] *= -1;
  }
}



/**
 * Put Discovery F4 specific initialization need here
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

  #ifdef BattMonitor
    pinMode(BATT_ANALOG_INPUT, INPUT_ANALOG);
  #endif

  // I2C setup
  Wire.begin(Port2Pin('B', 9), Port2Pin('B', 6)); // I2C1_SDA PB9, I2C1_SCL PB6

  HardCodedAxisCalibration();

  #if !defined(USE_USB_SERIAL)
    SerialUSB.begin();
  #endif

  #ifdef EnableLogging
    logInit();
    logPrintF("starting logging...\r\n");
  #endif
}

// called when eeprom is initialized
void initializePlatformSpecificAccelCalibration() {
// Accel Cal
  accelScaleFactor[XAXIS] = 1.0;
  runTimeAccelBias[XAXIS] = 0.0;
  accelScaleFactor[YAXIS] = 1.0;
  runTimeAccelBias[YAXIS] = 0.0;
  accelScaleFactor[ZAXIS] = 1.0;
  runTimeAccelBias[ZAXIS] = 0.0;
}


unsigned long previousMeasureCriticalSensorsTime = 0;
void measureCriticalSensors() {
  // read sensors not faster than every 1 ms
  if (currentTime - previousMeasureCriticalSensorsTime >= 1000) {
	measureGyroSum();
	measureAccelSum();

	previousMeasureCriticalSensorsTime = currentTime;
  }
}

#endif
