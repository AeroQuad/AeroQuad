#include "Gyroscope_Wii.h"

Gyroscope_Wii::Gyroscope_Wii() {
  // Add any required variable initialization here
  scaleFactor = 1.0; // Define the scale factor that converts to radians/second
}

void Gyroscope_Test::initialize() {
  // Add hardware initialization or setup here
  calibrate(); // Calibrate gyros after each power up, store zero values to EEPROM outside class
}

void Gyroscope_Test::measure() {
  // Replace code below with sensor measurement methodology
  for (byte axis = ROLL; axis < LASTAXIS; axis++)
    data[axis] = random(0, 1024) ;
  
  // Invert axis as needed here by switching gyroADC[] and zero[]
  // Axis definitions: roll right >0, pitch up >0, yaw right >0
  rate[0] = (data[0] - zero[0]) * scaleFactor;
  rate[1] = (data[1] - zero[1]) * scaleFactor;
  rate[2] = (data[2] - zero[2]) * scaleFactor;
}

void Gyroscope_Test::calibrate() {
  // Add calibration method for measurement when gyro is motionless
  for (byte axis = ROLL; axis < LASTAXIS; axis++)
    zero[axis] = random(510, 514); // simulate zero measurement around 512
}
