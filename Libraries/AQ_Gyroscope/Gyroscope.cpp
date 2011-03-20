#include <Gyroscope.h>

Gyroscope::Gyroscope() {
}

void Gyroscope::setZero(byte axis, int value) {
  zero[axis] = value;
}

const int Gyroscope::getZero(byte axis) {
  return zero[axis];
}
  
const float Gyroscope::getSmoothFactor() {
  return smoothFactor;
}

void Gyroscope::setSmoothFactor(float value) {
  smoothFactor = value;
}

void Gyroscope::setProcessTime(unsigned long value) {
  processTime = value;
}
  
const float Gyroscope::getRadPerSec(byte axis) {
  return rate[axis];
}