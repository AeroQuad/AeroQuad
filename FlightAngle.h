/*
  AeroQuad v2.2 - Feburary 2011
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
    angle[ROLL] = 0;
    angle[PITCH] = 0;
    angle[YAW] = 0;
    gyroAngle[ROLL] = 0;
    gyroAngle[PITCH] = 0;
  }
  
  virtual void initialize();
  virtual void calculate();
  virtual float getGyroUnbias(byte axis);
  virtual void calibrate();
 
  const float getData(byte axis) {
    return angle[axis];
  }
  
  const byte getType(void) {
    // This is set in each subclass to identify which algorithm used
    return type;
  }
};

/******************************************************/
/*************** Complementary Filter *****************/
/******************************************************/
// Originally authored by RoyLB
// http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
class FlightAngle_CompFilter : public FlightAngle {
private:
  float previousAngle[2];
  float filterTerm0[2];
  float filterTerm1[2];
  float filterTerm2[2];
  float timeConstantCF;

  void _initialize(byte axis) {
    previousAngle[axis] = accel.angleDeg(axis);
    filterTerm2[axis] = gyro.rateDegPerSec(axis);
    timeConstantCF = timeConstant; // timeConstant is a global variable read in from EEPROM
    // timeConstantCF should have been read in from set method, but needed common way for CF and KF to be initialized
    // Will take care of better OO implementation in future revision
  }
  
  float _calculate(byte axis, float newAngle, float newRate) {
    filterTerm0[axis] = (newAngle - previousAngle[axis]) * timeConstantCF *  timeConstantCF;
    filterTerm2[axis] += filterTerm0[axis] * G_Dt;
    filterTerm1[axis] = filterTerm2[axis] + (newAngle - previousAngle[axis]) * 2 *  timeConstantCF + newRate;
    previousAngle[axis] = (filterTerm1[axis] * G_Dt) + previousAngle[axis];
    return previousAngle[axis]; // This is actually the current angle, but is stored for the next iteration
  }

public:
  FlightAngle_CompFilter() : FlightAngle() {
    filterTerm0[ROLL] = 0;
    filterTerm1[ROLL] = 0;
    filterTerm0[PITCH] = 0;
    filterTerm1[PITCH] = 0;
    type = CF;
  }
  
  void initialize(void) {
    for (byte axis = ROLL; axis < YAW; axis++)
      _initialize(axis);
  }
  
  void calculate(void) {
    angle[ROLL] = _calculate(ROLL, accel.angleDeg(ROLL), gyro.rateDegPerSec(ROLL));
    angle[PITCH] = _calculate(PITCH, accel.angleDeg(PITCH), gyro.rateDegPerSec(PITCH));
  }
  
  float getGyroUnbias(byte axis) {
    return gyro.getFlightData(axis);
  }
  
  void calibrate(void) {}
};

/******************************************************/
/****************** Kalman Filter *********************/
/******************************************************/
// Originally authored by Tom Pycke
// http://tom.pycke.be/mav/71/kalman-filtering-of-imu-data
class FlightAngle_KalmanFilter : public FlightAngle {
private:
    float x_angle[2], x_bias[2];
    float P_00[2], P_01[2], P_10[2], P_11[2];	
    float Q_angle, Q_gyro;
    float R_angle;
    float y, S;
    float K_0, K_1;

    float _calculate(byte axis, float newAngle, float newRate) {
      x_angle[axis] += G_Dt * (newRate - x_bias[axis]);
      P_00[axis] +=  - G_Dt * (P_10[axis] + P_01[axis]) + Q_angle * G_Dt;
      P_01[axis] +=  - G_Dt * P_11[axis];
      P_10[axis] +=  - G_Dt * P_11[axis];
      P_11[axis] +=  + Q_gyro * G_Dt;
      
      y = newAngle - x_angle[axis];
      S = P_00[axis] + R_angle;
      K_0 = P_00[axis] / S;
      K_1 = P_10[axis] / S;
      
      x_angle[axis] +=  K_0 * y;
      x_bias[axis]  +=  K_1 * y;
      P_00[axis] -= K_0 * P_00[axis];
      P_01[axis] -= K_0 * P_01[axis];
      P_10[axis] -= K_1 * P_00[axis];
      P_11[axis] -= K_1 * P_01[axis];
      
      return x_angle[axis];
    }

public:
  FlightAngle_KalmanFilter() : FlightAngle() {
    for (byte axis = ROLL; axis < YAW; axis ++) {
      x_angle[axis] = 0;
      x_bias[axis] = 0;
      P_00[axis] = 0;
      P_01[axis] = 0;
      P_10[axis] = 0;
      P_11[axis] = 0;
    }
    type = KF;
  }
  
  void initialize(void) {
    Q_angle = 0.001;
    Q_gyro = 0.003;
    R_angle = 0.03;
  }
  
  void calculate(void) {
    angle[ROLL] = _calculate(ROLL, accel.angleDeg(ROLL), gyro.rateDegPerSec(ROLL));
    angle[PITCH] = _calculate(PITCH, accel.angleDeg(PITCH), gyro.rateDegPerSec(PITCH));
  }
  
  float getGyroUnbias(byte axis) {
    return gyro.getFlightData(axis);
  }

  void calibrate(void) {}
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
  float errorCourse;
  float errorRollPitch[3];
  float errorYaw[3];
  float scaledOmegaP[3];
  float scaledOmegaI[3];
  
  //  Roll and Pitch Compensation
  
  accelVector[XAXIS] = ax;
  accelVector[YAXIS] = ay;
  accelVector[ZAXIS] = az;

  // Calculate the magnitude of the accelerometer vector
  accelMagnitude = (sqrt(accelVector[XAXIS] * accelVector[XAXIS] + \
                         accelVector[YAXIS] * accelVector[YAXIS] + \
                         accelVector[ZAXIS] * accelVector[ZAXIS])) / oneG;
                         
  // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
  // accelWeight = constrain(1 - 4*abs(1 - accelMagnitude),0,1);
  
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  accelWeight = constrain(1 - 2 * abs(1 - accelMagnitude), 0, 1);
  
  vectorCrossProduct(&errorRollPitch[0], &accelVector[0], &dcmMatrix[6]);
  vectorScale(3, &omegaP[0], &errorRollPitch[0], kpRollPitch * accelWeight);
  
  vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], kiRollPitch * accelWeight);
  vectorAdd(3, omegaI, omegaI, scaledOmegaI);
  
  //  Yaw Compensation
  
  #ifdef COMPASS_INSTALLED
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
  angle[PITCH] = -asin(dcmMatrix[6]);
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

    kpRollPitch = 1.6;
    kiRollPitch = 0.005;
    
    kpYaw = -1.6;
    kiYaw = -0.005;
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
    return correctedRateVector[axis];
  }
  
  void calibrate() {};
  
};


/******************************************************/
/*********************** IMU **************************/
/******************************************************/
// Written by Sebastian O.H. Madgwick
// http://code.google.com/p/imumargalgorithm30042010sohm/

// Do not use, experimental code

class FlightAngle_IMU : public FlightAngle {
private:
  // System constants
  #define gyroMeasError 3.14159265358979f * (75.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
  #define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
  float SEq_1, SEq_2, SEq_3, SEq_4; // estimated orientation quaternion elements with initial conditions

  void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z) {
    // Local system variables
    float norm; // vector norm
    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
    float f_1, f_2, f_3; // objective function elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
    // Axulirary variables to avoid repeated calcualtions
    float halfSEq_1 = 0.5f * SEq_1;
    float halfSEq_2 = 0.5f * SEq_2;
    float halfSEq_3 = 0.5f * SEq_3;
    float halfSEq_4 = 0.5f * SEq_4;
    float twoSEq_1 = 2.0f * SEq_1;
    float twoSEq_2 = 2.0f * SEq_2;
    float twoSEq_3 = 2.0f * SEq_3;
    // Normalise the accelerometer measurement
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;
    // Compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
    J_12or23 = 2.0f * SEq_4;
    J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2;
    J_32 = 2.0f * J_14or21; // negated in matrix multiplication
    J_33 = 2.0f * J_11or24; // negated in matrix multiplication
    // Compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
    // Normalise the gradient
    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
    SEqHatDot_1 /= norm;
    SEqHatDot_2 /= norm;
    SEqHatDot_3 /= norm;
    SEqHatDot_4 /= norm;
    // Compute the quaternion derrivative measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
    // Compute then integrate the estimated quaternion derrivative
    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * G_Dt;
    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * G_Dt;
    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * G_Dt;
    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * G_Dt;
    // Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;
  }
  
public:
  FlightAngle_IMU() : FlightAngle() {}
  
  void initialize(void) {
    // estimated orientation quaternion elements with initial conditions
    SEq_1 = 1.0f; // w
    SEq_2 = 0.0f; // x
    SEq_3 = 0.0f; // y
    SEq_4 = 0.0f; // z
  }
  
  void calculate(void) {
    filterUpdate(gyro.rateRadPerSec(ROLL), gyro.rateRadPerSec(PITCH), gyro.rateRadPerSec(YAW), accel.getRaw(XAXIS), accel.getRaw(YAXIS), accel.getRaw(ZAXIS));
    angle[ROLL] = degrees(-asin((2 * SEq_2 * SEq_4) + (2 * SEq_1 * SEq_3)));
    angle[PITCH] = degrees(atan2((2 * SEq_3 * SEq_4) - (2 *SEq_1 * SEq_2), (2 * SEq_1 * SEq_1) + (2 *SEq_4 * SEq_4) - 1));
    angle[YAW] = degrees(atan2((2 * SEq_2 * SEq_3) - (2 * SEq_1 * SEq_4), (2 * SEq_1 * SEq_1) + (2 * SEq_2 * SEq_2) -1));
  }
  
  float getGyroUnbias(byte axis) {
    return gyro.getFlightData(axis);
  }

  void calibrate(void) {}
};

// ***********************************************************************
// ********************* MultiWii Kalman Filter **************************
// ***********************************************************************
// Original code by Alex at: http://radio-commande.com/international/triwiicopter-design/
// ************************************
// simplified IMU based on Kalman Filter
// inspired from http://starlino.com/imu_guide.html
// and http://www.starlino.com/imu_kalman_arduino.html
// with this algorithm, we can get absolute angles for a stable mode integration
// ************************************

class FlightAngle_MultiWii : public FlightAngle { 
private:
  int8_t signRzGyro;  
  float R;
  float RxEst; // init acc in stable mode
  float RyEst;
  float RzEst;
  float Axz,Ayz;           //angles between projection of R on XZ/YZ plane and Z axis (in Radian)
  float RxAcc,RyAcc,RzAcc;         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer       
  float RxGyro,RyGyro,RzGyro;        //R obtained from last estimated value and gyro movement
  float wGyro; // gyro weight/smooting factor
  float atanx,atany;
  float gyroFactor;
  //float meanTime; // **** Need to update this ***

public: 
  FlightAngle_MultiWii() : FlightAngle() {
    RxEst = 0; // init acc in stable mode
    RyEst = 0;
    RzEst = 1;
    wGyro = 50.0f; // gyro weight/smooting factor
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {}
  
  void calculate(void) {
    //get accelerometer readings in g, gives us RAcc vector
    RxAcc = accel.getRaw(ROLL);
    RyAcc = accel.getRaw(PITCH);
    RzAcc = accel.getRaw(YAW);
  
    //normalize vector (convert to a vector with same direction and with length 1)
    R = sqrt(square(RxAcc) + square(RyAcc) + square(RzAcc));
    RxAcc /= R;
    RyAcc /= R;  
    RzAcc /= R;  
  
    gyroFactor = G_Dt/83e6; //empirical, depends on WMP on IDG datasheet, tied of deg/ms sensibility
    
    //evaluate R Gyro vector
    if(abs(RzEst) < 0.1f) {
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      RxGyro = RxEst;
      RyGyro = RyEst;
      RzGyro = RzEst;
    }
    else {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last REst
      //Convert ADC value for to physical units
      //For gyro it will return  deg/ms (rate of rotation)
      atanx = atan2(RxEst,RzEst);
      atany = atan2(RyEst,RzEst);
    
      Axz = atanx + gyro.getRaw(ROLL)  * gyroFactor;  // convert ADC value for to physical units
      Ayz = atany + gyro.getRaw(PITCH) * gyroFactor; // and get updated angle according to gyro movement
    
      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
      signRzGyro = ( cos(Axz) >=0 ) ? 1 : -1;
  
      //reverse calculation of RwGyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
      RxGyro = sin(Axz) / sqrt( 1 + square(cos(Axz)) * square(tan(Ayz)) );
      RyGyro = sin(Ayz) / sqrt( 1 + square(cos(Ayz)) * square(tan(Axz)) );        
      RzGyro = signRzGyro * sqrt(1 - square(RxGyro) - square(RyGyro));
    }
    
    //combine Accelerometer and gyro readings
    RxEst = (RxAcc + wGyro* RxGyro) / (1.0 + wGyro);
    RyEst = (RyAcc + wGyro* RyGyro) / (1.0 + wGyro);
    RzEst = (RzAcc + wGyro* RzGyro) / (1.0 + wGyro);
  
    angle[ROLL]  =  180/PI * Axz;
    angle[PITCH] =  180/PI * Ayz;
  }
  
  float getGyroUnbias(byte axis) {
    return gyro.getFlightData(axis);
  }

  void calibrate(void) {}
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

  void initialize(void) {
    calibrate();
  }

  void calculate(void) {   
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
  void initialize(void) {
    calibrate();
  }

  void calculate(void) {

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

