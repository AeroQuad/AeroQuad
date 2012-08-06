//  Place board in refrigerator for 1 hour
//  Remove, and execute this sketch
//    1st set of readings taken immeadiately while board is still cold
//    2nd set of readings taken 1 hour later when board has warmed to room temperature
//
//  Slopes and intercepts of the bias vs temperature funciton are calculated, and the 
//  lines of code required are generated.  These lines need to be copied and pasted
//  into AreoQuad.h

#include <EEPROM.h>

#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include <AQMath.h>


#define LASTCHANNEL 6

#include "AeroQuad.h"
#include "PID.h"



//#include <AQMath.h>

//#define AeroQuad_v18
//#define AeroQuad_Mini
#define AeroQuadMega_v2
//#define AeroQuadMega_v21


#ifdef AeroQuad_v18

  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>
  /**
   * Put AeroQuad_v18 specific intialization need here
   */
  void initPlatform() {

    Wire.begin();
    TWBR = 12;
  }
#endif

#ifdef AeroQuad_Mini
  #include <Device_I2C.h>

  // Gyroscope declaration
  #define ITG3200_ADDRESS_ALTERNATE
  #include <Gyroscope_ITG3200.h>

  /**
   * Put AeroQuad_Mini specific intialization need here
   */
  void initPlatform() {

    Wire.begin();
    TWBR = 12;
  }
#endif

#ifdef AeroQuadMega_v2
  #include <Device_I2C.h>

  // Gyroscope declaration
  #include <Gyroscope_ITG3200.h>

  /**
   * Put AeroQuadMega_v2 specific intialization need here
   */
  void initPlatform() {

    Wire.begin();
    TWBR = 12;
  }

  /**
   * Measure critical sensors
   */
  void measureCriticalSensors() {
    measureGyroSum();
  }
#endif

#ifdef AeroQuadMega_v21
  #include <Device_I2C.h>

  // Gyroscope declaration
  #define ITG3200_ADDRESS_ALTERNATE
  #include <Gyroscope_ITG3200_9DOF.h>

  /**
   * Put AeroQuadMega_v21 specific intialization need here
   */
  void initPlatform() {

    Wire.begin();
    TWBR = 12;
  }
#endif


#include <Accelerometer.h>
#include <Receiver.h>
#include "DataStorage.h"

float gyroSampleRate = 2000;
float numberOfGyroSamples = 1000;

float gyroBias1[3]     = {0, 0, 0};
float gyroTemperature1 = 0;
float gyroBias2[3]     = {0, 0, 0};
float gyroTemperature2 = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("====== Start calibration process, wait! ======");
  readEEPROM(); // defined in DataStorage.h
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all parameters
    Serial.println("Init EEPROM since it's a different version");
    initializeEEPROM();
    writeEEPROM();
  }
  
  
  initPlatform();
  delay(1000);
  initializeGyro();
  
  ////////////////////////////////
  // Get samples at temperature1
  ////////////////////////////////
  Serial.println("Begin 1st Gyro Measurements...");
  for (int i = 0; i < numberOfGyroSamples; i++)
  {
    measureGyro();
    gyroBias1[XAXIS] += gyroRate[XAXIS];
    gyroBias1[YAXIS] += gyroRate[YAXIS];
    gyroBias1[ZAXIS] += gyroRate[ZAXIS];
    readGyroTemp();
    gyroTemperature1 += gyroTemperature;
    delayMicroseconds(gyroSampleRate);
  } 
  
  gyroBias1[XAXIS] /= numberOfGyroSamples;
  gyroBias1[YAXIS] /= numberOfGyroSamples;
  gyroBias1[ZAXIS] /= numberOfGyroSamples;
  gyroTemperature1 /= numberOfGyroSamples;
  
  Serial.println();
  Serial.print("Gyro Temperature Reading: ");  Serial.println(gyroTemperature1, 2);
  Serial.println();
  Serial.println("End 1st Gyro Measurements");
  
  ////////////////////////////////
  // Time delay for temperature
  // Stabilizaiton
  ////////////////////////////////
  
  Serial.println();
  Serial.println("Waiting for 15 minutes for temps to rise...");
  Serial.println();
  //delay(300000);    // Number of mSec in 5 minutes (for testing)
  delay(900000);   // Number of mSec in 15 minutes
  
  ////////////////////////////////
  // Get samples at temperature2
  ////////////////////////////////
  Serial.println("Begin 2nd Gyro Measurements...");
  for (int i = 0; i < numberOfGyroSamples; i++)
  {
    measureGyro();
    gyroBias2[XAXIS] += gyroRate[XAXIS];
    gyroBias2[YAXIS] += gyroRate[YAXIS];
    gyroBias2[ZAXIS] += gyroRate[ZAXIS];
    readGyroTemp();
    gyroTemperature2 += gyroTemperature;
    delayMicroseconds(gyroSampleRate);
  }
  
  gyroBias2[XAXIS] /= numberOfGyroSamples;
  gyroBias2[YAXIS] /= numberOfGyroSamples;
  gyroBias2[ZAXIS] /= numberOfGyroSamples;
  gyroTemperature2 /= numberOfGyroSamples;
  
  Serial.println();
  Serial.print("Gyro Temperature Reading: ");  Serial.println(gyroTemperature2, 2);
  Serial.println();
  Serial.println("End 2nd Gyro Measurements");

  gyroTempBiasSlope[XAXIS] = (gyroBias2[XAXIS] - gyroBias1[XAXIS]) / (gyroTemperature2 - gyroTemperature1);
  gyroTempBiasSlope[YAXIS] = (gyroBias2[YAXIS] - gyroBias1[YAXIS]) / (gyroTemperature2 - gyroTemperature1);
  gyroTempBiasSlope[ZAXIS] = (gyroBias2[ZAXIS] - gyroBias1[ZAXIS]) / (gyroTemperature2 - gyroTemperature1);
  
  gyroTempBiasIntercept[XAXIS] = gyroBias2[XAXIS] - gyroTempBiasSlope[XAXIS] * gyroTemperature2;
  gyroTempBiasIntercept[YAXIS] = gyroBias2[YAXIS] - gyroTempBiasSlope[YAXIS] * gyroTemperature2;
  gyroTempBiasIntercept[ZAXIS] = gyroBias2[ZAXIS] - gyroTempBiasSlope[ZAXIS] * gyroTemperature2;
  
  Serial.println();
  
  Serial.print("gyroTCBiasSlope[3]      = { ");
  Serial.print(gyroTempBiasSlope[XAXIS], 10); 
  Serial.print(", ");
  Serial.print(gyroTempBiasSlope[YAXIS], 10);
  Serial.print(", ");
  Serial.print(gyroTempBiasSlope[ZAXIS], 10);
  Serial.println(" };");
  
  Serial.print("gyroTCBiasIntercept[3]  = { ");
  Serial.print(gyroTempBiasIntercept[XAXIS], 10);
  Serial.print(", ");
  Serial.print(gyroTempBiasIntercept[YAXIS], 10);
  Serial.print(", ");
  Serial.print(gyroTempBiasIntercept[ZAXIS], 10);
  Serial.println(" };");
  
  writeFloat(gyroTempBiasSlope[XAXIS], GYRO_ROLL_TEMP_BIAS_SLOPE_ADR);
  writeFloat(gyroTempBiasSlope[YAXIS], GYRO_PITCH_TEMP_BIAS_SLOPE_ADR);
  writeFloat(gyroTempBiasSlope[ZAXIS], GYRO_YAW_TEMP_BIAS_SLOPE_ADR);
  
  writeFloat(gyroTempBiasIntercept[XAXIS], GYRO_ROLL_TEMP_BIAS_INTERCEPT_ADR);
  writeFloat(gyroTempBiasIntercept[YAXIS], GYRO_PITCH_TEMP_BIAS_INTERCEPT_ADR);
  writeFloat(gyroTempBiasIntercept[ZAXIS], GYRO_YAW_TEMP_BIAS_INTERCEPT_ADR);
  
  Serial.println();
  Serial.println("EEPROM write complete");
}

void loop()
{
}
  
  
