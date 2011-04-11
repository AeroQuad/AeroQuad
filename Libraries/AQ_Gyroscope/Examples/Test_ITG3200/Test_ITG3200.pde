/**
	Example for ITG3200 (I2C 3-axis gyroscope sensor) library.
	Code by Ivan Todorovic.
*/

#include <Wire.h>
#include <APM_ADC.h>    // @see Kenny, Arduino IDE compiliation bug
#include <AQMath.h>
#include <Device_I2C.h>
#include <Gyroscope_ITG3200.h>
#include <Axis.h>

unsigned long timer;

Gyroscope_ITG3200 gyro;

void setup()
{
  Serial.begin(115200);
  Serial.println("Gyroscope library test (ITG3200)");

  Wire.begin();

  gyro.initialize();
  gyro.calibrate();
  timer = millis();
}

void loop(void) 
{
  if((millis() - timer) > 10) // 100Hz
  {
    timer = millis();
    gyro.measure();
    
    Serial.print("Roll: ");
    Serial.print(degrees(gyro.getRadPerSec(ROLL)));
    Serial.print(" Pitch: ");
    Serial.print(degrees(gyro.getRadPerSec(PITCH)));
    Serial.print(" Yaw: ");
    Serial.print(degrees(gyro.getRadPerSec(YAW)));
    Serial.print(" Heading: ");
    Serial.print(degrees(gyro.getHeading()));
    Serial.println();
  }
}

