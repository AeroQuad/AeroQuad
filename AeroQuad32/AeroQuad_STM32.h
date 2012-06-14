#undef CameraControl
#undef OSD

#define __STM32__
#define DEBUG_INIT
tSerial &Serial = SERIAL_VAR;

#ifdef BOARD_aeroquad32
	#define STM32_BOARD_TYPE "aeroquad32"
	#define LED_Green  Port2Pin('E', 6)
	#define LED_Red    Port2Pin('E', 5)
	#define LED_Yellow LED_Red
#endif
#ifdef BOARD_aeroquad32mini
	#define STM32_BOARD_TYPE "aeroquad32 mini"
	#define LED_Green  13 // PA3
	#define LED_Red    13
	#define LED_Yellow 13
#endif
#ifdef BOARD_freeflight
	#define STM32_BOARD_TYPE "Free Flight"
	#define LED_Green  Port2Pin('B', 4)
	#define LED_Red    Port2Pin('B', 3)
	#define LED_Yellow LED_Red
	#define ITG3200_ADDRESS_ALTERNATE
#endif
#ifdef BOARD_discovery_f4
	#define STM32_BOARD_TYPE "Discovery F4"
	#define LED_Green  Port2Pin('D', 12)
	#define LED_Red    Port2Pin('D', 14)
	#define LED_Yellow Port2Pin('D', 13)
#endif

#include <Device_I2C.h>

// Gyroscope declaration
#ifdef SENSORBOARD_ALA42
	#include <Gyroscope_ITG3200_ala42.h>
#else
	//#include <Gyroscope_ITG3200.h>
	#include <Platform_MPU6000.h>
	#include <Gyroscope_MPU6000.h>
#endif

// Accelerometer declaration
#ifdef BOARD_freeflight
  #include <Accelerometer_ADXL345_9DOF.h>
#else
  //#include <Accelerometer_BMA180.h>
  //#include <Accelerometer_BMA180_ala42.h>
  #include <Accelerometer_MPU6000.h>
#endif

// Receiver Declaration
#ifndef Receiver_PPM
#define RECEIVER_STM32
#else
#define RECEIVER_STM32PPM
#endif

// Motor declaration
#define MOTOR_STM32

// heading mag hold declaration
#ifdef HeadingMagHold
	#ifdef SENSORBOARD_ALA42
		#define HMC5843
	#else
		#define HMC5883L
	#endif
#endif

// Altitude declaration
#ifdef AltitudeHoldBaro
	#ifdef SENSORBOARD_ALA42
	  #define BMP085
	#else
  	#define MS5611
  #endif
#endif

#ifdef AltitudeHoldRangeFinder
	#define XLMAXSONAR
#endif

// Battery Monitor declaration
#ifdef BattMonitor
	#define ADC_NUMBER_OF_BITS	12
	#define BATT_AREF			3.3		// V
	#define BATT_MAX_DIGITAL	((float)(1<<ADC_NUMBER_OF_BITS))

	#ifdef BOARD_freeflight
		#define BATT_R_HIGH			10.0	// kOhm
		#define BATT_R_LOW			1.0		// kOhm
		#define BATT_ANALOG_INPUT	Port2Pin('A', 4)
		#define BATT_DIODE_LOSS		0.76
	#else
		#define BATT_R_HIGH			10.0	// kOhm
		#define BATT_R_LOW			1.5		// kOhm
		#define BATT_ANALOG_INPUT	Port2Pin('C', 0)
		#define BATT_DIODE_LOSS		0.0
	#endif
	#define BattDefaultConfig DEFINE_BATTERY(0, BATT_ANALOG_INPUT, ((BATT_AREF / BATT_MAX_DIGITAL) * (BATT_R_HIGH + BATT_R_LOW) / BATT_R_LOW), BATT_DIODE_LOSS, BM_NOPIN, 0, 0)
#endif

#ifdef OSD
	#define MAX7456_OSD
#endif



/**
 * Put AeroQuadSTM32 specific initialization need here
 */

void HardCodedAxisCalibration()
{
  if(accelScaleFactor[XAXIS] == 1.0 && accelScaleFactor[YAXIS] == 1.0 && accelScaleFactor[ZAXIS] == 1.0) {
#ifdef BOARD_freeflight
    accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = -0.038;
#else
    #ifdef _AEROQUAD_PLATFORM_MPU6000_H_
      accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = 0.00119;
    #else
      accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = 0.0048;
    #endif
  	accelScaleFactor[YAXIS] *= -1;
  	accelScaleFactor[ZAXIS] *= -1;
#endif
  }
}



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

#ifdef DEBUG_INIT
  Serial.println("\r\nAeroQuad STM32, board type " STM32_BOARD_TYPE ", build date " __DATE__ " "__TIME__);
#endif


// I2C setup
#ifdef BOARD_aeroquad32
  Wire.begin(Port2Pin('B', 7), Port2Pin('B', 6)); // I2C1_SDA PB7, I2C1_SCL PB6
#endif
#ifdef BOARD_aeroquad32mini
  Wire.begin(16, 17); // I2C2_SDA PB11, I2C2_SCL PB10
#endif
#ifdef BOARD_freeflight
  Wire.begin(Port2Pin('B', 11), Port2Pin('B', 10)); // I2C1_SDA PB9, I2C1_SCL PB6
#endif
#ifdef BOARD_discovery_f4
  Wire.begin(Port2Pin('B', 9), Port2Pin('B', 6)); // I2C1_SDA PB9, I2C1_SCL PB6
#endif



#ifdef _AEROQUAD_PLATFORM_MPU6000_H_
  initializeMPU6000Sensors();
#endif

  HardCodedAxisCalibration();

#ifdef DEBUG_INIT
  Serial.println("STM32 init done\r\n");
#endif
}


void SignalAlive(int frameCounter) {
  digitalWrite(LED_Green, (frameCounter/50) & 1);
}

unsigned long previousMeasureCriticalSensorsTime = 0;
void measureCriticalSensors() {
  // read sensors not faster than every 1 ms
  if (currentTime - previousMeasureCriticalSensorsTime >= 1000) {
#ifdef _AEROQUAD_PLATFORM_MPU6000_H_
    readMPU6000Sensors();
#endif
    measureAccelSum();
    measureGyroSum();
    SignalAlive(frameCounter);

    previousMeasureCriticalSensorsTime = currentTime;
  }
}
