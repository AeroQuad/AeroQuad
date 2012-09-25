#ifndef _PLATFORM_AEROQUAD32_H_
	#define _PLATFORM_AEROQUAD32_H_

	static byte stm32_motor_mapping[] = {
		Port2Pin('C',  9),
		Port2Pin('C',  8),
		Port2Pin('C',  7),
		Port2Pin('C',  6),
		Port2Pin('A', 15),
		Port2Pin('B',  3),
		Port2Pin('B',  4),
		Port2Pin('B',  5)
	};

#ifdef RECEIVER_STM32PPM
  static byte receiverPinPPM = Port2Pin('D', 15);
#elif defined ReceiverSBUS
#else
	static byte receiverPin[] = {
		Port2Pin('D', 12),
		Port2Pin('D', 13),
		Port2Pin('D', 14),
		Port2Pin('D', 15),
		Port2Pin('E',  9),
		Port2Pin('E', 11),
		Port2Pin('E', 13),
		Port2Pin('E', 14)
	};
#endif


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


	#include <Device_I2C.h>

	#include <Gyroscope_MPU6000.h>
	#include <Accelerometer_MPU6000.h>

	// heading mag hold declaration
	#ifdef HeadingMagHold
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

		#define BATT_R_HIGH       10.0	// kOhm
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


	/**
	 * Put AeroQuadSTM32 specific initialization need here
	 */

	void HardCodedAxisCalibration()
	{
		initSensorsZeroFromEEPROM();

		if(   (accelScaleFactor[XAXIS] == 1.0 && accelScaleFactor[YAXIS] == 1.0 && accelScaleFactor[ZAXIS] == 1.0)
			 || (accelScaleFactor[XAXIS] == 0.0 && accelScaleFactor[YAXIS] == 0.0 && accelScaleFactor[ZAXIS] == 0.0)
		) {
			accelScaleFactor[XAXIS] =  0.00119;
			accelScaleFactor[YAXIS] = -0.00119;
			accelScaleFactor[ZAXIS] = -0.00119;
			storeSensorsZeroToEEPROM();
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

	    pinMode(BATT_ANALOG_INPUT, INPUT_ANALOG);
	    pinMode(A1, INPUT_ANALOG);
	    pinMode(A2, INPUT_ANALOG);
	    pinMode(A3, INPUT_ANALOG);
	    pinMode(A4, INPUT_ANALOG);
	    pinMode(A5, INPUT_ANALOG);
	    pinMode(A6, INPUT_ANALOG);

	  #ifdef DEBUG_INIT
	    long t0 = micros();
	    Serial.println("\r\nAeroQuad STM32, board type " STM32_BOARD_TYPE ", build date " __DATE__ " "__TIME__);
	    long t1 = micros();
	    Serial.println(t1-t0);
	  #endif


	  // I2C setup
		Wire.begin(Port2Pin('B', 7), Port2Pin('B', 6)); // I2C1_SDA PB7, I2C1_SCL PB6

		HardCodedAxisCalibration();

	  #ifdef DEBUG_INIT
	  	Serial.println("STM32 init done\r\n");
	  #endif

	  #if !defined(USE_USB_SERIAL)
	  	SerialUSB.begin();
	  #endif
	}


	void SignalAlive(int frameCounter) {
		digitalWrite(LED_Green, (frameCounter/50) & 1);
	}

	unsigned long previousMeasureCriticalSensorsTime = 0;
	void measureCriticalSensors() {
		// read sensors not faster than every 1 ms
		if (currentTime - previousMeasureCriticalSensorsTime >= 1000) {
			measureGyroSum();
			measureAccelSum();

			SignalAlive(frameCounter);

			previousMeasureCriticalSensorsTime = currentTime;
		}
	}

#endif
