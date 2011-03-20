#ifndef _AEROQUAD_GYROSCOPE_H_
#define _AEROQUAD_GYROSCOPE_H_

class Gyroscope {
protected:
  float scaleFactor;
  float rate[3];
  float zero[3];
  unsigned long lastMeasuredTime;
  unsigned long processTime;
  
public:  
  Gyroscope();

  virtual boolean present();
  virtual void calibrate();
  virtual void initialize();
  virtual void measure();

  void setZero(byte axis, float value);
  const float getZero(byte axis);
  void setProcessTime(unsigned long time, unsigned long offset);
  const float getRadPerSec(byte axis);
};
#endif