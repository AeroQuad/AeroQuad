/**
	Example for ITG3200 (I2C 3-axis gyroscope sensor) library.
	Code by Ivan Todorovic.
*/

#include <Wire.h>       // @see Kenny, Arduino IDE compiliation bug
#include <APM_ADC.h>    // @see Kenny, Arduino IDE compiliation bug
#include <AQMath.h>
#include <Device_I2C.h>
#include <Gyroscope_IDG_IDZ500.h>
#include <Axis.h>

unsigned long timer;

Gyroscope_IDG_IDZ500 gyro;

void setup()
{
  Serial.begin(115200);
  Serial.println("Gyroscope library test (IDG_IDZ500)");

  gyro.setAref(3.3);
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

