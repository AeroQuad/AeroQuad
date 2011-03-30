/**
	Example for ITG3200 (I2C 3-axis gyroscope sensor) library.
	Code by Ivan Todorovic.
*/

#include <Wire.h>
#include "Gyroscope_ITG3200.h"

#define ROLL 0
#define PITCH 1
#define YAW 2

unsigned long timer;
Gyroscope_ITG3200 gyro;

void setup()
{
	Serial.begin(115200);
	Serial.println("Gyroscope library test (ITG3200)");

	if (gyro.detectPresence(1) != 0)
		Serial.println("Gyroscope not found!");
	else
		gyro.initialize();
	timer = millis();
}

void loop(void) 
{
	if((millis() - timer) > 50) // 20Hz
	{
		timer = millis();
		gyro.measure();

		Serial.print("Roll: ");
		Serial.print(degrees(gyro.getRadPerSec(ROLL)));
		Serial.print(" Pitch: ");
		Serial.print(degrees(gyro.getRadPerSec(PITCH)));
		Serial.print(" Yaw: ");
		Serial.print(degrees(gyro.getRadPerSec(YAW)));
		Serial.println();
	}
}

