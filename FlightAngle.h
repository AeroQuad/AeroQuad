/*
  AeroQuad v2.0 - July 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
  An Open Source Arduino based quadrocopter.
 
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
  
  FlightAngle(void) {
    angle[ROLL] = 0;
    angle[PITCH] = 0;
    angle[YAW] = 0;
    gyroAngle[ROLL] = 0;
    gyroAngle[PITCH] = 0;
  }
  
  virtual void initialize();
  virtual void calculate();
  virtual float getGyroAngle(byte axis);
  
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
    previousAngle[axis] = (fixed)accel.angleDeg(axis);
    filterTerm2[axis] = (fixed)gyro.rateDegPerSec(axis);
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
    for (axis = ROLL; axis < YAW; axis++)
      _initialize(axis);
  }
  
  void calculate(void) {
    angle[ROLL] = _calculate(ROLL, accel.angleDeg(ROLL), gyro.rateDegPerSec(ROLL));
    angle[PITCH] = _calculate(PITCH, accel.angleDeg(PITCH), gyro.rateDegPerSec(PITCH));
  }

  float getGyroAngle(byte axis) {
    gyroAngle[axis] += gyro.rateDegPerSec(axis) * G_Dt;
  }
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
    for (axis = ROLL; axis ++; axis < YAW) {
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
  
  float getGyroAngle(byte axis) {
    gyroAngle[axis] += gyro.rateDegPerSec(axis) * G_Dt;
  }
};

/******************************************************/
/*********************** DCM **************************/
/******************************************************/
// Written by William Premerlani
// http://diydrones.com/profiles/blogs/dcm-imu-theory-first-draft
class FlightAngle_DCM : public FlightAngle {
private:
  float dt;
  float Gyro_Gain_X;
  float Gyro_Gain_Y;
  float Gyro_Gain_Z;
  float DCM_Matrix[3][3];
  float Update_Matrix[3][3];
  float Temporary_Matrix[3][3];
  float Accel_Vector[3];
  float Accel_Vector_unfiltered[3];
  float Gyro_Vector[3];
  float Omega_Vector[3];
  float Omega_P[3];
  float Omega_I[3];
  float Omega[3];
  float errorRollPitch[3];
  float errorYaw[3];
  float errorCourse;
  float COGX; //Course overground X axis
  float COGY; //Course overground Y axis
  
  //Computes the dot product of two vectors
  float Vector_Dot_Product(float vector1[3],float vector2[3]) {
    float op=0;

    for(int c=0; c<3; c++)
      op+=vector1[c]*vector2[c];
    return op; 
  }

  //Computes the cross product of two vectors
  void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]) {
    vectorOut[0]= (v1[1]*v2[2]) - (v1[2]*v2[1]);
    vectorOut[1]= (v1[2]*v2[0]) - (v1[0]*v2[2]);
    vectorOut[2]= (v1[0]*v2[1]) - (v1[1]*v2[0]);
  }

  //Multiply the vector by a scalar. 
  void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2) {
    for(int c=0; c<3; c++)
      vectorOut[c]=vectorIn[c]*scale2; 
  }

  void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]){
    for(int c=0; c<3; c++)
      vectorOut[c]=vectorIn1[c]+vectorIn2[c];
  }

  /********* MATRIX FUNCTIONS *****************************************/
  //Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
  void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]) {
    float op[3]; 
    for(int x=0; x<3; x++) {
      for(int y=0; y<3; y++) {
        for(int w=0; w<3; w++)
         op[w]=a[x][w]*b[w][y];
        mat[x][y]=0;
        mat[x][y]=op[0]+op[1]+op[2];
        float test=mat[x][y];
      }
    }
  }
  
public:
  FlightAngle_DCM():FlightAngle() {
    for (byte i=0; i<3; i++) {
      Accel_Vector[i] = 0; //Store the acceleration in a vector
      Accel_Vector_unfiltered[i] = 0; //Store the acceleration in a vector
      Gyro_Vector[i] = 0;//Store the gyros rutn rate in a vector
      Omega_Vector[i] = 0; //Corrected Gyro_Vector data
      Omega_P[i] = 0;//Omega Proportional correction
      Omega_I[i] = 0;//Omega Integrator
      Omega[i] = 0;
      errorRollPitch[i]= 0;
      errorYaw[i]= 0;
    }
    DCM_Matrix[0][0] = 1;
    DCM_Matrix[0][1] = 0;
    DCM_Matrix[0][2] = 0;
    DCM_Matrix[1][0] = 0;
    DCM_Matrix[1][1] = 1;
    DCM_Matrix[1][2] = 0;
    DCM_Matrix[2][0] = 0;
    DCM_Matrix[2][1] = 0;
    DCM_Matrix[2][2] = 1;
    Update_Matrix[0][0] = 0;
    Update_Matrix[0][1] = 1;
    Update_Matrix[0][2] = 2;
    Update_Matrix[1][0] = 3;
    Update_Matrix[1][1] = 4;
    Update_Matrix[1][2] = 5;
    Update_Matrix[2][0] = 6;
    Update_Matrix[2][1] = 7;
    Update_Matrix[2][2] = 8;
    Temporary_Matrix[0][0] = 0;
    Temporary_Matrix[0][1] = 0;
    Temporary_Matrix[0][2] = 0;
    Temporary_Matrix[1][0] = 0;
    Temporary_Matrix[1][1] = 0;
    Temporary_Matrix[1][2] = 0;
    Temporary_Matrix[2][0] = 0;
    Temporary_Matrix[2][1] = 0;
    Temporary_Matrix[2][2] = 0;
    
    errorCourse = 0;
    COGX = 0; //Course overground X axis
    COGY = 1; //Course overground Y axis    
    dt = 0;
    // IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 2.93mV/ADC step => 0.8/3.33 = 0.683
    Gyro_Gain_X = radians(0.683);
    Gyro_Gain_Y = radians(0.683);
    Gyro_Gain_Z = radians(0.683);
    type = DCM;
  }
  
  void initialize(void) {    
  }
  
  void calculate(void) {
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
  }
  
  float getGyroAngle(byte axis) {
    return degrees(Omega[axis]);
  }
  
  void Normalize(void) {
    float error=0;
    float temporary[3][3];
    float renorm=0;
    
    error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19
  
    Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
    Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19
    
    Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
    Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19
    
    Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20
    
    renorm= .5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
    Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
    
    renorm= .5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
    Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
    
    renorm= .5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
    Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
  }

  void Drift_correction(void) {
    //Compensation the Roll, Pitch and Yaw drift. 
    float errorCourse;
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float Accel_magnitude;
    float Accel_weight;
    
    //*****Roll and Pitch***************
  
    // Calculate the magnitude of the accelerometer vector
    // Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
    // Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
    // Accel_weight = constrain(1 - 4*abs(1 - Accel_magnitude),0,1);
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    // Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
    Accel_weight = 1.0;
  
    Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
    Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
    
    Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
    Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);
    
    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading 
    /*if (MAGNETOMETER == 1) {
      errorCourse= (DCM_Matrix[0][0]*APM_Compass.Heading_Y) - (DCM_Matrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
      Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    
      Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
      Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
    
      Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
      Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
    }*/
  }

  /*void Accel_adjust(void) {
    // ADC : Voltage reference 3.0V / 10bits(1024 steps) => 2.93mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 2.93mV/ADC step => 330/0.8 = 102
    #define GRAVITY 102 //this equivalent to 1G in the raw data coming from the accelerometer 
    #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

    Accel_Vector[1] += Accel_Scale(speed_3d*Omega[2]);  // Centrifugal force on Acc_y = GPS_speed*GyroZ
    Accel_Vector[2] -= Accel_Scale(speed_3d*Omega[1]);  // Centrifugal force on Acc_z = GPS_speed*GyroY
  }*/

  void Matrix_update(void) {
    Gyro_Vector[0]=Gyro_Gain_X * -gyro.getData(PITCH); //gyro x roll
    Gyro_Vector[1]=Gyro_Gain_Y * gyro.getData(ROLL); //gyro y pitch
    Gyro_Vector[2]=Gyro_Gain_Z * gyro.getData(YAW); //gyro Z yaw
    
    Accel_Vector[0]=-accel.getData(ROLL); // acc x
    Accel_Vector[1]=accel.getData(PITCH); // acc y
    Accel_Vector[2]=accel.getData(ZAXIS); // acc z
    
    // Low pass filter on accelerometer data (to filter vibrations)
    //Accel_Vector[0]=Accel_Vector[0]*0.5 + (float)read_adc(3)*0.5; // acc x
    //Accel_Vector[1]=Accel_Vector[1]*0.5 + (float)read_adc(4)*0.5; // acc y
    //Accel_Vector[2]=Accel_Vector[2]*0.5 + (float)read_adc(5)*0.5; // acc z
    
    Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);//adding integrator
    Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]);//adding proportional
    
    //Accel_adjust();//adjusting centrifugal acceleration. // Not used for quadcopter
    
    Update_Matrix[0][0]=0;
    Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
    Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
    Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
    Update_Matrix[1][1]=0;
    Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
    Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
    Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
    Update_Matrix[2][2]=0;

    //Serial.println(DCM_Matrix[2][0], 6);

    Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
  
    for(int x=0; x<3; x++) {  //Matrix Addition (update)
      for(int y=0; y<3; y++)
        DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    }
  }

  void Euler_angles(void) {
      angle[ROLL] = degrees(asin(-DCM_Matrix[2][0]));
      angle[PITCH] = degrees(atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]));
      angle[YAW] = degrees(atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]));
  }
};

/******************************************************/
/*********************** IMU **************************/
/******************************************************/
// Written by Sebastian O.H. Madgwick
// http://code.google.com/p/imumargalgorithm30042010sohm/

// Do note use, experimental code

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
  
  float getGyroAngle(byte axis) {
    gyroAngle[axis] += gyro.rateDegPerSec(axis) * G_Dt;
  }
};


