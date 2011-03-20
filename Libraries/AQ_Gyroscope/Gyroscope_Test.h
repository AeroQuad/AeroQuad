#ifndef _AEROQUAD_GYROSCOPE_TEST_H_
#define _AEROQUAD_GYROSCOPE_TEST_H_

#include <WProgram.h>
#include <Gyroscope.h>

class Gyroscope_Test : public Gyroscope {
public:
  boolean present(void);
  void calibrate(void);	
  void initialize(void);
  void measure(void);

private:
  int data[3]; // raw data from sensor
};
#endif