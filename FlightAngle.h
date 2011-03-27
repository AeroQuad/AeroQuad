/*
  AeroQuad v2.3 - March 2011
  www.AeroQuad.com
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

// This class is responsible for calculating vehicle attitude
class FlightAngle {
public:
  #define CF 0
  #define KF 1
  #define DCM 2
  #define IMU 3
  byte type;
  float angle[3];
  float gyroAngle[2];
  float correctedRateVector[3];
  float earthAccel[3];
  
  FlightAngle(void) {
    for (byte axis = ROLL; axis < LASTAXIS; axis++)
      angle[axis] = 0.0;
    //angle[ROLL] = 0;
    //angle[PITCH] = 0;
    //angle[YAW] = 0;
    gyroAngle[ROLL] = 0;
    gyroAngle[PITCH] = 0;
  }
  
  virtual void initialize(float hdgX, float hdgY);
  virtual void calculate(float rollRate,           float pitchRate,     float yawRate,       \
                         float longitudinalAccel,  float lateralAccel,  float verticalAccel, \
                         float oneG,               float magX,          float magY);
  virtual float getGyroUnbias(byte axis);
  virtual void calibrate();
 
  // returns the angle of a specific axis in SI units (radians)
  const float getData(byte axis) {
    return angle[axis];
  }
  // return heading as +PI/-PI
  const float getHeading(byte axis) {
    return(angle[axis]);
  }
  
  // This really needs to be in Radians to be consistent
  // I'll fix later - AKA
  // returns heading in degrees as 0-360
  const float getDegreesHeading(byte axis) {
    float tDegrees;
    
    tDegrees = degrees(angle[axis]);
    if (tDegrees < 0.0)
      return (tDegrees + 360.0);
    else
      return (tDegrees);
  }
  
  const byte getType(void) {
    // This is set in each subclass to identify which algorithm used
    return type;
  }
};

////////////////////////////////////////////////////////////////////////////////
// DCM
////////////////////////////////////////////////////////////////////////////////

// Written by William Premerlani
// Modified by Jose Julio for multicopters
// http://diydrones.com/profiles/blogs/dcm-imu-theory-first-draft
// Optimizations done by Jihlein
// http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=12286&viewfull=1#post12286

class FlightAngle_DCM : public FlightAngle {
private:
  float dcmMatrix[9];
  float omegaP[3];
  float omegaI[3];
  float omega[3];
  float errorCourse;
  float kpRollPitch;
  float kiRollPitch;
  float kpYaw;
  float kiYaw;

////////////////////////////////////////////////////////////////////////////////
// Matrix Update
////////////////////////////////////////////////////////////////////////////////

void matrixUpdate(float p, float q, float r) 
{
  float rateGyroVector[3];
  float temporaryMatrix[9];
  float updateMatrix[9];
  
  rateGyroVector[ROLL]  = p;
  rateGyroVector[PITCH] = q;
  rateGyroVector[YAW]   = r;
  
  vectorSubtract(3, &omega[ROLL], &rateGyroVector[ROLL], &omegaI[ROLL]);
  vectorSubtract(3, &correctedRateVector[ROLL], &omega[ROLL], &omegaP[ROLL]); 
  
  //Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter
  
  updateMatrix[0] =  0;
  updateMatrix[1] = -G_Dt * correctedRateVector[YAW];    // -r
  updateMatrix[2] =  G_Dt * correctedRateVector[PITCH];  //  q
  updateMatrix[3] =  G_Dt * correctedRateVector[YAW];    //  r
  updateMatrix[4] =  0;
  updateMatrix[5] = -G_Dt * correctedRateVector[ROLL];   // -p
  updateMatrix[6] = -G_Dt * correctedRateVector[PITCH];  // -q
  updateMatrix[7] =  G_Dt * correctedRateVector[ROLL];   //  p
  updateMatrix[8] =  0; 

  matrixMultiply(3, 3, 3, temporaryMatrix, dcmMatrix, updateMatrix); 
  matrixAdd(3, 3, dcmMatrix, dcmMatrix, temporaryMatrix);
}

////////////////////////////////////////////////////////////////////////////////
// Normalize
////////////////////////////////////////////////////////////////////////////////

void normalize(void) 
{
  float error=0;
  float temporary[9];
  float renorm=0;
  
  error= -vectorDotProduct(3, &dcmMatrix[0], &dcmMatrix[3]) * 0.5;         // eq.18

  vectorScale(3, &temporary[0], &dcmMatrix[3], error);                     // eq.19
  vectorScale(3, &temporary[3], &dcmMatrix[0], error);                     // eq.19
  
  vectorAdd(6, &temporary[0], &temporary[0], &dcmMatrix[0]);               // eq.19
  
  vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);           // eq.20
  
  for(byte v=0; v<9; v+=3) {
    renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));  // eq.21
    vectorScale(3, &dcmMatrix[v], &temporary[v], renorm);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Drift Correction
////////////////////////////////////////////////////////////////////////////////

void driftCorrection(float ax, float ay, float az, float oneG, float magX, float magY) 
{
  //  Compensation of the Roll, Pitch and Yaw drift. 
  float accelMagnitude;
  float accelVector[3];
  float accelWeight;
  float errorRollPitch[3];
#ifdef HeadingMagHold  
  float errorCourse;
  float errorYaw[3];
  float scaledOmegaP[3];
#endif  
  float scaledOmegaI[3];
  
  //  Roll and Pitch Compensation
  
  accelVector[XAXIS] = accelVector[XAXIS]*0.05 + ax*0.05;
  accelVector[YAXIS] = accelVector[YAXIS]*0.05 + ay*0.05;
  accelVector[ZAXIS] = accelVector[ZAXIS]*0.05 + az*0.05;

  // Calculate the magnitude of the accelerometer vector
  accelMagnitude = (sqrt(accelVector[XAXIS] * accelVector[XAXIS] + \
                         accelVector[YAXIS] * accelVector[YAXIS] + \
                         accelVector[ZAXIS] * accelVector[ZAXIS])) / oneG;
                         
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // accelWeight = constrain(1 - 4 * abs(1 - accelMagnitude), 0, 1);
  
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  accelWeight = constrain(1 - 2 * abs(1 - accelMagnitude), 0, 1);
  
//  accelWeight = 1.0;
  
  vectorCrossProduct(&errorRollPitch[0], &accelVector[0], &dcmMatrix[6]);
  vectorScale(3, &omegaP[0], &errorRollPitch[0], kpRollPitch);// * accelWeight);
  
  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], kiRollPitch);// * accelWeight);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);
  
  //  Yaw Compensation
  
  #ifdef HeadingMagHold
    errorCourse = (dcmMatrix[0] * magY) - (dcmMatrix[3] * magX);
    vectorScale(3, errorYaw, &dcmMatrix[6], errorCourse);
  
    vectorScale(3, &scaledOmegaP[0], &errorYaw[0], kpYaw);
    vectorAdd(3, omegaP, omegaP, scaledOmegaP);
  
    vectorScale(3, &scaledOmegaI[0] ,&errorYaw[0], kiYaw);
    vectorAdd(3, omegaI, omegaI, scaledOmegaI);
  #else
    omegaP[YAW] = 0.0;
    omegaI[YAW] = 0.0;
  #endif
}

////////////////////////////////////////////////////////////////////////////////
// Accel Adjust
////////////////////////////////////////////////////////////////////////////////

/*void Accel_adjust(void) {
  // ADC : Voltage reference 3.0V / 10bits(1024 steps) => 2.93mV/ADC step
  // ADXL335 Sensitivity(from datasheet) => 330mV/g, 2.93mV/ADC step => 330/0.8 = 102
  #define GRAVITY 102 //this equivalent to 1G in the raw data coming from the accelerometer 
  #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

  accelVector[1] += Accel_Scale(speed_3d*omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
  accelVector[2] -= Accel_Scale(speed_3d*omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
}*/

////////////////////////////////////////////////////////////////////////////////
// Euler Angles
////////////////////////////////////////////////////////////////////////////////

void eulerAngles(void)
{
  angle[ROLL]  =  atan2(dcmMatrix[7], dcmMatrix[8]);
  angle[PITCH] =  -asin(dcmMatrix[6]);
  angle[YAW]   =  atan2(dcmMatrix[3], dcmMatrix[0]);
} 
  
////////////////////////////////////////////////////////////////////////////////
// Earth Axis Accels
////////////////////////////////////////////////////////////////////////////////

void earthAxisAccels(float ax, float ay, float az, float oneG)
{
  float accelVector[3];
  
  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;
  
  earthAccel[XAXIS] = vectorDotProduct(3, &dcmMatrix[0], &accelVector[0]);
  earthAccel[YAXIS] = vectorDotProduct(3, &dcmMatrix[3], &accelVector[0]);
  earthAccel[ZAXIS] = vectorDotProduct(3, &dcmMatrix[6], &accelVector[0]) + oneG;
} 
  
public:
  FlightAngle_DCM():FlightAngle() {}
  
////////////////////////////////////////////////////////////////////////////////
// Initialize DCM
////////////////////////////////////////////////////////////////////////////////

  void initialize(float hdgX, float hdgY) 
  {
    for (byte i=0; i<3; i++) {
      omegaP[i] = 0;
      omegaI[i] = 0;
    }
    dcmMatrix[0] =  hdgX;
    dcmMatrix[1] = -hdgY;
    dcmMatrix[2] =  0;
    dcmMatrix[3] =  hdgY;
    dcmMatrix[4] =  hdgX;
    dcmMatrix[5] =  0;
    dcmMatrix[6] =  0;
    dcmMatrix[7] =  0;
    dcmMatrix[8] =  1;

    // Original from John
//    kpRollPitch = 1.6;
//    kiRollPitch = 0.005;
    
//    kpYaw = -1.6;
//    kiYaw = -0.005;
/*    
    // released in 2.2
    kpRollPitch = 1.0;
    kiRollPitch = 0.002;
    
    kpYaw = -1.0;
    kiYaw = -0.002;
*/
    kpRollPitch = 0.5;
    kiRollPitch = 0.001;
    
    kpYaw = -0.5;
    kiYaw = -0.001;
  }
  
////////////////////////////////////////////////////////////////////////////////
// Calculate DCM
////////////////////////////////////////////////////////////////////////////////

  void calculate(float rollRate,            float pitchRate,      float yawRate,  \
                 float longitudinalAccel,   float lateralAccel,   float verticalAccel, \
                 float oneG,                float magX,           float magY) {
    
    matrixUpdate(rollRate, pitchRate, yawRate); 
    normalize();
    driftCorrection(longitudinalAccel, lateralAccel, verticalAccel, oneG, magX, magY);
    eulerAngles();
    earthAxisAccels(longitudinalAccel, lateralAccel, verticalAccel, oneG);
  }
  
  float getGyroUnbias(byte axis) {
//    return correctedRateVector[axis];
    return omega[axis];
  }
  
  void calibrate() {};
  
};

// ***********************************************************************
// ********************* CHR6DM "null" Filter ***************************
// ***********************************************************************
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
class FlightAngle_CHR6DM : public FlightAngle {
private:

float zeroRoll;
float zeroPitch;

public:
  FlightAngle_CHR6DM() : FlightAngle() {}

  void initialize(float hdgX, float hdgY) {
    calibrate();
  }

  void calculate(float rollRate,           float pitchRate,     float yawRate,       \
                 float longitudinalAccel,  float lateralAccel,  float verticalAccel, \
                 float oneG,               float magX,          float magY) {
    angle[ROLL]  =  chr6dm.data.roll - zeroRoll;
    angle[PITCH] =  chr6dm.data.pitch - zeroPitch;
    CHR_RollAngle = angle[ROLL]; //ugly since gotta access through accel class
    CHR_PitchAngle = angle[PITCH];
  }
  
   void calibrate(void) {
    zeroRoll = chr6dm.data.roll;
    zeroPitch = chr6dm.data.pitch;
  }
  
  float getGyroUnbias(byte axis) {
    return gyro.getFlightData(axis);
  }

};
#endif

// ***********************************************************************
// ********************* CHR6DM "null" Filter ***************************
// ***********************************************************************
#ifdef CHR6DM_FAKE_FLIGHTANGLE
class FlightAngle_CHR6DM_Fake : public FlightAngle {
private:

float zeroRoll;
float zeroPitch;

public:
  FlightAngle_CHR6DM_Fake() : FlightAngle() {}

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(float hdgX, float hdgY) {
    calibrate();
  }

  void calculate(float rollRate,           float pitchRate,     float yawRate,       \
                 float longitudinalAccel,  float lateralAccel,  float verticalAccel, \
                 float oneG,               float magX,          float magY) {
    angle[ROLL]  =  0 - zeroRoll;
    angle[PITCH] =  0 - zeroPitch;
    CHR_RollAngle = angle[ROLL]; //ugly since gotta access through accel class
    CHR_PitchAngle = angle[PITCH];
  }

   void calibrate(void) {
    zeroRoll = 0;
    zeroPitch = 0;
  }
  
  float getGyroUnbias(byte axis) {
    return gyro.getFlightData(axis);
  }

  
};
#endif

