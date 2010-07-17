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
  float angle[3];
  
  FlightAngle(void) {
    angle[ROLL] = 0;
    angle[PITCH] = 0;
    angle[YAW] = 0;
  }
  
  virtual void initialize();
  virtual void calculate();
  
  const float getData(byte axis) {
    return angle[axis];
  }
};

/******************************************************/
/*************** Complementary Filter *****************/
/******************************************************/
// Originally authored by RoyLB
// At http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
class FlightAngle_CompFilter : public FlightAngle {
private:
  float dt;
  float previousAngle;
  float newAngle;
  float newRate;
  float filterTerm0;
  float filterTerm1;
  float filterTerm2;
  float timeConstantCF;

  void _initialize(byte axis) {
    previousAngle = accel.angleDeg(axis);
    filterTerm2 = gyro.rateDegPerSec(axis);
    dt = AIdT;
    timeConstantCF = timeConstant; // timeConstant is a global variable read in from EEPROM
    // timeConstantCF should have been read in from set method, but needed common way for CF and KF to be initialized
    // Will take care of better OO implementation in future revision
  }
  
  float _calculate(float newAngle, float newRate) {
    filterTerm0 = (newAngle - previousAngle) * timeConstant *  timeConstantCF;
    filterTerm2 += filterTerm0 * dt;
    filterTerm1 = filterTerm2 + (newAngle - previousAngle) * 2 *  timeConstantCF + newRate;
    previousAngle = (filterTerm1 * dt) + previousAngle;
    return previousAngle; // This is actually the current angle, but is stored for the next iteration
  }

public:
  FlightAngle_CompFilter() : FlightAngle() {
    dt = 0;
    filterTerm0 = 0;
    filterTerm1 = 0;
  }
  
  void initialize(void) {
    for (axis = ROLL; axis < YAW; axis++)
      _initialize(axis);
  }
  
  void calculate(void) {
    angle[ROLL] = _calculate(accel.angleDeg(ROLL), gyro.rateDegPerSec(ROLL));
    angle[PITCH] = _calculate(accel.angleDeg(PITCH), gyro.rateDegPerSec(PITCH));
  }
};

/******************************************************/
/****************** Kalman Filter *********************/
/******************************************************/
class FlightAngle_KalmanFilter : public FlightAngle {
private:
    float x_angle, x_bias;
    float P_00, P_01, P_10, P_11;	
    float Q_angle, Q_gyro;
    float R_angle;
    float dt, y, S;
    float K_0, K_1;

    float _calculate(float newAngle, float newRate) {
      x_angle += dt * (newRate - x_bias);
      P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
      P_01 +=  - dt * P_11;
      P_10 +=  - dt * P_11;
      P_11 +=  + Q_gyro * dt;
      
      y = newAngle - x_angle;
      S = P_00 + R_angle;
      K_0 = P_00 / S;
      K_1 = P_10 / S;
      
      x_angle +=  K_0 * y;
      x_bias  +=  K_1 * y;
      P_00 -= K_0 * P_00;
      P_01 -= K_0 * P_01;
      P_10 -= K_1 * P_00;
      P_11 -= K_1 * P_01;
      
      return x_angle;
    }

public:
  FlightAngle_KalmanFilter() : FlightAngle() {
    x_angle = 0;
    x_bias = 0;
    P_00 = 0;
    P_01 = 0;
    P_10 = 0;
    P_11 = 0;
  }
  
  void initialize(void) {
    Q_angle = 0.001;
    Q_gyro = 0.003;
    R_angle = 0.03;
    dt = AIdT;
  }
  
  void calculate(void) {
    angle[ROLL] = _calculate(accel.angleDeg(ROLL), gyro.rateDegPerSec(ROLL));
    angle[PITCH] = _calculate(accel.angleDeg(PITCH), gyro.rateDegPerSec(PITCH));
  }    
};

/******************************************************/
/*********************** DCM **************************/
/******************************************************/
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
    Gyro_Vector[0]=Gyro_Gain_X * gyro.getData(ROLL); //gyro x roll
    Gyro_Vector[1]=Gyro_Gain_Y * gyro.getData(PITCH); //gyro y pitch
    Gyro_Vector[2]=Gyro_Gain_Z * gyro.getData(YAW); //gyro Z yaw
    
    Accel_Vector[0]=accel.getData(ROLL); // acc x
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
  
    Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
  
    for(int x=0; x<3; x++) {  //Matrix Addition (update)
      for(int y=0; y<3; y++)
        DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
    }
  }

  void Euler_angles(void) {
      angle[ROLL] = asin(-DCM_Matrix[2][0]);
      angle[PITCH] = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
      angle[YAW] = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
  }

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
  FlightAngle_DCM() :FlightAngle() {
    float Accel_Vector[3] = {0,0,0}; //Store the acceleration in a vector
    float Accel_Vector_unfiltered[3] = {0,0,0}; //Store the acceleration in a vector
    float Gyro_Vector[3] = {0,0,0};//Store the gyros rutn rate in a vector
    float Omega_Vector[3] = {0,0,0}; //Corrected Gyro_Vector data
    float Omega_P[3] = {0,0,0};//Omega Proportional correction
    float Omega_I[3] = {0,0,0};//Omega Integrator
    float Omega[3] = {0,0,0};
    float DCM_Matrix[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    float Update_Matrix[3][3] = {{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
    float Temporary_Matrix[3][3]={{0,0,0},{0,0,0},{0,0,0}};
    float errorRollPitch[3]= {0,0,0};
    float errorYaw[3]= {0,0,0};
    
    errorCourse = 0;
    COGX = 0; //Course overground X axis
    COGY = 1; //Course overground Y axis    
    dt = 0;
    // IDG500 Sensitivity (from datasheet) => 2.0mV/º/s, 2.93mV/ADC step => 0.8/3.33 = 0.683
    Gyro_Gain_X = radians(0.683);
    Gyro_Gain_Y = radians(0.683);
    Gyro_Gain_Z = radians(0.683);
  }
  
  void initialize(void) {    
  }
  
  void calculate(void) {
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
  }
};
