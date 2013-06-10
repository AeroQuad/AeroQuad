#ifndef _PLATFORM_FREEFLIGHT_H_
	#define _PLATFORM_FREEFLIGHT_H_

	static byte stm32_motor_mapping[] = {
		Port2Pin('B',  6),
		Port2Pin('B',  7),
		Port2Pin('B',  8),
		Port2Pin('B',  9),
		Port2Pin('A',  8),
		Port2Pin('A', 11)
	};

#ifdef RECEIVER_STM32PPM
  static byte receiverPinPPM = Port2Pin('A', 0);
#else
	static byte receiverPin[] = {
		Port2Pin('A',  0),
		Port2Pin('A',  1),
		Port2Pin('A',  2),
		Port2Pin('A',  3),
		Port2Pin('A',  6),
		Port2Pin('A',  7),
		Port2Pin('B',  0),
		Port2Pin('B',  1)
	};
#endif


	#define STM32_BOARD_TYPE "Free Flight"
	#define LED_Green  Port2Pin('B', 4)
	#define LED_Red    Port2Pin('B', 3)
	#define LED_Yellow LED_Red
	#define ITG3200_ADDRESS_ALTERNATE


	#include <Device_I2C.h>

	#include <Gyroscope_ITG3200.h>
	#include <Accelerometer_ADXL345_9DOF.h>


	// heading mag hold declaration
	#ifdef HeadingMagHold
		#define HMC5883L
	#endif

	// Altitude declaration
	#ifdef AltitudeHoldBaro
		#define BMP085
	#endif


	// Battery Monitor declaration
	#ifdef BattMonitor
		#define BATT_AREF			3.3		// V

		#define BATT_R_HIGH			10.0	// kOhm
		#define BATT_R_LOW			1.0		// kOhm
		#define BATT_ANALOG_INPUT	Port2Pin('A', 4)
		#define BATT_DIODE_LOSS		0.76
		#define BattDefaultConfig DEFINE_BATTERY(0, BATT_ANALOG_INPUT, (BATT_AREF * (BATT_R_HIGH + BATT_R_LOW) / BATT_R_LOW), BATT_DIODE_LOSS, BM_NOPIN, 0, 0)
	#endif


	void HardCodedAxisCalibration()
	{
    initSensorsZeroFromEEPROM();
		if(   (accelScaleFactor[XAXIS] == 1.0 && accelScaleFactor[YAXIS] == 1.0 && accelScaleFactor[ZAXIS] == 1.0)
			 || (accelScaleFactor[XAXIS] == 0.0 && accelScaleFactor[YAXIS] == 0.0 && accelScaleFactor[ZAXIS] == 0.0)
		) {
			accelScaleFactor[XAXIS] = accelScaleFactor[YAXIS] = accelScaleFactor[ZAXIS] = -0.038;
			//accelScaleFactor[YAXIS] *= -1;
			//accelScaleFactor[ZAXIS] *= -1;

      storeSensorsZeroToEEPROM();
		}
	}



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

	#ifdef BattMonitor
		pinMode(BATT_ANALOG_INPUT, INPUT_ANALOG);
	#endif


	#ifdef DEBUG_INIT
		Serial.println("\r\nAeroQuad STM32, board type " STM32_BOARD_TYPE ", build date " __DATE__ " "__TIME__);
	#endif


	// I2C setup
		Wire.begin(Port2Pin('B', 11), Port2Pin('B', 10)); // I2C1_SDA PB11, I2C1_SCL PB10


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
			measureGyroSum();
			measureAccelSum();

			SignalAlive(frameCounter);

			previousMeasureCriticalSensorsTime = currentTime;
		}
	}
#endif
