#ifndef _PLATFORM_AEROQUAD32_H_

#define _PLATFORM_AEROQUAD32_H_

#define STM32_BOARD_TYPE "aeroquad32"
#define LED_Green  Port2Pin('E', 6)
#define LED_Red    Port2Pin('E', 5)
#define LED_Yellow LED_Red

#define BATT_ANALOG_INPUT	Port2Pin('C', 0)
#define A1       Port2Pin('B',0)
#define A2       Port2Pin('C',4)
#define A3       Port2Pin('B',1)
#define A4       Port2Pin('C',5)
#define A5       Port2Pin('C',2)
#define A6       Port2Pin('C',3)

// external LED drivers
#define PLED1 Port2Pin('D',  7)
#define PLED2 Port2Pin('E',  0)
#define PLED3 Port2Pin('E',  1)
#define PLED4 Port2Pin('D',  4) 

#include <Device_I2C.h>

#include <Gyroscope_MPU6000.h>
#include <Accelerometer_MPU6000.h>

// heading mag hold declaration
#ifdef HeadingMagHold
  #include <Compass.h>
  #define HMC5883L
#endif

// Altitude declaration
#ifdef AltitudeHoldBaro
  #define MS5611
#endif

#ifdef AltitudeHoldRangeFinder
  #define XLMAXSONAR
#endif

// Battery Monitor declaration
#ifdef BattMonitor
  #define BATT_AREF         3.3		// V
  #define BATT_R_HIGH       10.0		// kOhm
  #define BATT_R_LOW        1.5		// kOhm
  #define BATT_DIODE_LOSS		0.0
  #define BattDefaultConfig DEFINE_BATTERY(0, BATT_ANALOG_INPUT, (BATT_AREF * (BATT_R_HIGH + BATT_R_LOW) / BATT_R_LOW), BATT_DIODE_LOSS, BM_NOPIN, 0, 0)
#endif

#ifdef OSD
  #define MAX7456_OSD
#endif

#ifdef CameraControl
  #define CameraControl_STM32
#endif

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

  pinMode(BATT_ANALOG_INPUT, INPUT_ANALOG);
  pinMode(A1, INPUT_ANALOG);
  pinMode(A2, INPUT_ANALOG);
  pinMode(A3, INPUT_ANALOG);
  pinMode(A4, INPUT_ANALOG);
  pinMode(A5, INPUT_ANALOG);
  pinMode(A6, INPUT_ANALOG);

  pinMode(PLED1, OUTPUT);
  pinMode(PLED2, OUTPUT);
  pinMode(PLED3, OUTPUT);
  pinMode(PLED4, OUTPUT);

  // I2C setup
  Wire.begin(Port2Pin('B', 7), Port2Pin('B', 6)); // I2C1_SDA PB7, I2C1_SCL PB6

  #if !defined(USE_USB_SERIAL)
    SerialUSB.begin();
  #endif
  
  switch (flightConfigType) 
  {
    case OCTO_X :
    case OCTO_PLUS :
    case OCTO_X8 :
	  LASTMOTOR = 8;
	  break;
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
  accelScaleFactor[XAXIS] = 0.0011970000;
  accelScaleFactor[YAXIS] = -0.0012050000;
  accelScaleFactor[ZAXIS] = -0.0011770000;
  #ifdef HeadingMagHold
    magBias[XAXIS]  = 152.000000;
    magBias[YAXIS]  = 24.000000;
    magBias[ZAXIS]  = 16.500000;
  #endif

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
