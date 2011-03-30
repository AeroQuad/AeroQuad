#ifndef _AEROQUAD_GYROSCOPE_WII_H_
#define _AEROQUAD_GYROSCOPE_WII_H_

#include <WProgram.h>
#include <Gyroscope.h>

class Gyroscope_Wii : public Gyroscope {
private:
  int data[3]; // raw data from sensor

  public:
  void initialize(void);
  void measure(void);
  void calibrate(void);
};
#endif