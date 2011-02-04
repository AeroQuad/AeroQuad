/*
  AeroQuad v2.1 - January 2011
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
};

/******************************************************/
/*********************** DCM **************************/
/******************************************************/
// Written by William Premerlani
// Modified by Jose Julio for multicopters
// http://diydrones.com/profiles/blogs/dcm-imu-theory-first-draft
// Optimizations done by Jihlein
// http://aeroquad.com/showthread.php?991-AeroQuad-Flight-Software-v2.0&p=12286&viewfull=1#post12286
class FlightAngle_DCM : public FlightAngle {
private:
  float dt;
  float Gyro_Gain;  // jihlein; Replaced X, Y, and Z gyro gains with single gain
  float DCM_Matrix[9];
  float Accel_Vector[3];
  float Omega_Vector[3];
  float Omega_P[3];
  float Omega_I[3];
  float Omega[3];
  float errorCourse;
  float COGX; //Course overground X axis
  float COGY; //Course overground Y axis
  float Kp_ROLLPITCH;
  float Ki_ROLLPITCH;


  //**********************************************************************************************
  //
  //  Matrix Update
  //
  //**********************************************************************************************
  void matrixUpdate(void) 
  {
    float gyroVector[3];
    gyroVector[0]=-(gyro.getData(PITCH) * Gyro_Gain); //gyro y roll
    gyroVector[1]=gyro.getData(ROLL) * Gyro_Gain; //gyro x pitch
    gyroVector[2]=gyro.getData(YAW) * Gyro_Gain; //gyro Z yaw
    vectorAdd(3, &Omega[0], &gyroVector[0], &Omega_I[0]);   // adding integrator
    vectorAdd(3, &Omega_Vector[0], &Omega[0], &Omega_P[0]);  // adding proportional
    
    // Low pass filter on accelerometer data (to filter vibrations)
    Accel_Vector[0]=Accel_Vector[0]*0.6 + (float)-accel.rateG(ROLL)*100.0; // acc x
    Accel_Vector[1]=Accel_Vector[1]*0.6 + (float)accel.rateG(PITCH)*100.0; // acc y
    Accel_Vector[2]=Accel_Vector[2]*0.6 + (float)accel.rateG(ZAXIS)*100.0; // acc z
    
    float updateMatrix[9];
    updateMatrix[0] =  0;
    updateMatrix[1] = -G_Dt*Omega_Vector[2];  // -z
    updateMatrix[2] =  G_Dt*Omega_Vector[1];  //  y
    updateMatrix[3] =  G_Dt*Omega_Vector[2];  //  z
    updateMatrix[4] =  0;
    updateMatrix[5] = -G_Dt*Omega_Vector[0];  // -x
    updateMatrix[6] = -G_Dt*Omega_Vector[1];  // -y
    updateMatrix[7] =  G_Dt*Omega_Vector[0];  //  x
    updateMatrix[8] =  0;

    float temporaryMatrix[9];
    matrixMultiply(3, 3, 3, temporaryMatrix, DCM_Matrix, updateMatrix); //a*b=c
    matrixAdd(3, 3, DCM_Matrix, DCM_Matrix, temporaryMatrix);
  }

  //**********************************************************************************************
  //
  //  Normalize
  //
  //**********************************************************************************************
  void normalize(void) 
  {
    float temporary[9];
    float renorm=0;
    
    float error= -vectorDotProduct(3, &DCM_Matrix[0], &DCM_Matrix[3])*.5;         // eq.18
  
    vectorScale(3, &temporary[0], &DCM_Matrix[3], error);                   // eq.19
    vectorScale(3, &temporary[3], &DCM_Matrix[0], error);                   // eq.19
    
    vectorAdd(6, &temporary[0], &temporary[0], &DCM_Matrix[0]);             // eq.19
    
    vectorCrossProduct(&temporary[6],&temporary[0],&temporary[3]);          // c= a x b //eq.20
    
    for(byte v=0; v<9; v+=3) {
      renorm = 0.5 *(3 - vectorDotProduct(3, &temporary[v],&temporary[v]));   // eq.21
      vectorScale(3, &DCM_Matrix[v], &temporary[v], renorm);
    }
  }

  //**********************************************************************************************
  //
  //  Drift Correction
  //
  //**********************************************************************************************
  void driftCorrection(void) 
  {
    //Compensation the Roll, Pitch and Yaw drift. 
    //float        errorCourse;
    //static float Scaled_Omega_P[3];
    float scaledOmegaI[3];
    float errorRollPitch[3];
    
    //*****Roll and Pitch***************
  
    // Calculate the magnitude of the accelerometer vector
    // Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
    // Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
    // Weight for accelerometer info (<0.75G = 0.0, 1G = 1.0 , >1.25G = 0.0)
    // Accel_weight = constrain(1 - 4*abs(1 - Accel_magnitude),0,1);
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    // Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);
  
    vectorCrossProduct(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[6]); //adjust the ground of reference
    // Limit max errorRollPitch to limit max Omega_P and Omega_I
    #define MAX_ERROR 50
    errorRollPitch[0] = constrain(errorRollPitch[0],-MAX_ERROR,MAX_ERROR);
    errorRollPitch[1] = constrain(errorRollPitch[1],-MAX_ERROR,MAX_ERROR);
    errorRollPitch[2] = constrain(errorRollPitch[2],-MAX_ERROR,MAX_ERROR);
    vectorScale(3, &Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH);
    
    vectorScale(3, &scaledOmegaI[0], &errorRollPitch[0], Ki_ROLLPITCH);
    vectorAdd(3, Omega_I, Omega_I, scaledOmegaI);
    
    //*****YAW***************
    // We make the gyro YAW drift correction based on compass magnetic heading 
    /*if (MAGNETOMETER == 1) {
  	  float errorYaw[3];
  
      errorCourse= (DCM_Matrix[0][0]*APM_Compass.Heading_Y) - (DCM_Matrix[1][0]*APM_Compass.Heading_X);  //Calculating YAW error
      Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
    
      Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);
      Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
    
      Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);
      Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
    }*/
  }

  //**********************************************************************************************
  //
  //  Euler Angles
  //
  //**********************************************************************************************
  void eulerAngles(void)
  {
    angle[ROLL] =  degrees(asin(-DCM_Matrix[6]));
    angle[PITCH] = degrees(atan2(DCM_Matrix[7],DCM_Matrix[8]));
    angle[YAW] =   degrees(atan2(DCM_Matrix[3],DCM_Matrix[0]));
  } 
  
public:
  FlightAngle_DCM():FlightAngle() {}
  
  void initialize(void) 
  {
    for (byte i=0; i<3; i++) {
      Accel_Vector[i]            = 0;  // Store the acceleration in a vector
      Omega_Vector[i]            = 0;  // Corrected Gyro_Vector data
      Omega_P[i]                 = 0;  // Omega Proportional correction
      Omega_I[i]                 = 0;  // Omega Integrator
      Omega[i]                   = 0;
    }
    DCM_Matrix[0]       = 1;
    DCM_Matrix[1]       = 0;
    DCM_Matrix[2]       = 0;
    DCM_Matrix[3]       = 0;
    DCM_Matrix[4]       = 1;
    DCM_Matrix[5]       = 0;
    DCM_Matrix[6]       = 0;
    DCM_Matrix[7]       = 0;
    DCM_Matrix[8]       = 1;

    errorCourse = 0;
    COGX = 0; //Course overground X axis
    COGY = 1; //Course overground Y axis    
    dt = 0;
    Gyro_Gain = radians(gyro.getScaleFactor());
    type = DCM;
    Kp_ROLLPITCH = 0.0014;
    Ki_ROLLPITCH = 0.00000012; // was 0.00000015
  }
  
  void calculate(void) {
    matrixUpdate(); 
    normalize();
    driftCorrection();
    eulerAngles();
  }
  
  float getGyroUnbias(byte axis) { 
    if (axis == ROLL)
      return degrees(Omega[1]);
    if (axis == PITCH)
      return degrees(-Omega[0]);
    else
      return degrees(Omega[2]);
  }
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

