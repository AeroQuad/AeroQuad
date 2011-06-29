/*
  AeroQuad v2.4.2 - June 2011
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

#define G_2_MPS2(g) (g * 9.80665)
#define MPS2_2_G(m) (m * 0.10197162)

////////////////////////////////////////////////////////////////////////////////
//
//  Constant Definitions
//
//  General:
//
//    a = (2 * tau)/T
//      tau = filter time constant
//      T   = sample time
//
//  Lag Filter:
//
//    gx1 = 1/(1+a)
//    gx2 = 1/(1+a)
//    gx3 = (1-a)/(1+a)
//
//  Washout Filter:
//
//    gx1 = a/(1+a)
//    gx2 = -a/(1+a)
//    gx3 = (1-a)/(1+a)
//
////////////////////////////////////////////////////////////////////////////////
struct firstOrderData
{
  float gx1;
  float gx2;
  float gx3;
  float lastInput;
  float lastOutput;
} firstOrder[3];

float computeFirstOrder(float currentInput, struct firstOrderData *filterParameters)
{
  filterParameters->lastOutput = filterParameters->gx1 * currentInput + 
                                 filterParameters->gx2 * filterParameters->lastInput - 
                                 filterParameters->gx3 * filterParameters->lastOutput;
           
  filterParameters->lastInput = currentInput;
    
  return filterParameters->lastOutput;
}

#define AX_LAG 0
#define AY_LAG 1
#define AZ_LAG 2

void setupFilters(float oneG)
{
/*  
  // 1 sec @ 100hz - logging filter
  firstOrder[AX_LAG].gx1 =  0.005;
  firstOrder[AX_LAG].gx2 =  0.005;
  firstOrder[AX_LAG].gx3 = -0.990;
  firstOrder[AX_LAG].lastInput =  0.0;
  firstOrder[AX_LAG].lastOutput = 0.0;
  
  firstOrder[AY_LAG].gx1 =  0.005;
  firstOrder[AY_LAG].gx2 =  0.005;
  firstOrder[AY_LAG].gx3 = -0.990;
  firstOrder[AY_LAG].lastInput =  0.0;
  firstOrder[AY_LAG].lastOutput = 0.0;
  
  firstOrder[AZ_LAG].gx1 =  0.005;
  firstOrder[AZ_LAG].gx2 =  0.005;
  firstOrder[AZ_LAG].gx3 = -0.990;
  firstOrder[AZ_LAG].lastInput =  -oneG;
  firstOrder[AZ_LAG].lastOutput = -oneG;
*/
  // .3 second @ 250hz
  firstOrder[AX_LAG].gx1 =  0.007;
  firstOrder[AX_LAG].gx2 =  0.007;
  firstOrder[AX_LAG].gx3 = -0.987;
  firstOrder[AX_LAG].lastInput =  0.0;
  firstOrder[AX_LAG].lastOutput = 0.0;
  
  firstOrder[AY_LAG].gx1 =  0.007;
  firstOrder[AY_LAG].gx2 =  0.007;
  firstOrder[AY_LAG].gx3 = -0.987;
  firstOrder[AY_LAG].lastInput =  0.0;
  firstOrder[AY_LAG].lastOutput = 0.0;
  
  firstOrder[AZ_LAG].gx1 =  0.007;
  firstOrder[AZ_LAG].gx2 =  0.007;
  firstOrder[AZ_LAG].gx3 = -0.987;
  firstOrder[AZ_LAG].lastInput =  -oneG;
  firstOrder[AZ_LAG].lastOutput = -oneG;
}


// Low pass filter, kept as regular C function for speed
float filterSmooth(float currentData, float previousData, float smoothFactor) {
  if (smoothFactor != 1.0) // only apply time compensated filter if smoothFactor is applied
    return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor)); 
  else
    return currentData; // if smoothFactor == 1.0, do not calculate, just bypass!
}

// ***********************************************************************
// *********************** Median Filter Class ***************************
// ***********************************************************************
// Median filter currently not used, but kept if needed for the future
// To declare use: MedianFilter filterSomething;

#define DATASIZE 25

class MedianFilter {
public: 
  float data[DATASIZE], sortData[DATASIZE];
  int dataIndex;
  MedianFilter(void) {}

  void initialize(void) {
    for (int index = 0; index < DATASIZE; index++) {
      data[index] = 0;
      sortData[index] = 0;
    }
    dataIndex = 0;
  }
  
  const float filter(float newData) {
    int temp, j; // used to sort array

    // Insert new data into raw data array round robin style
    data[dataIndex] = newData;
    if (dataIndex < (DATASIZE-1)) 
      dataIndex++;
    else 
      dataIndex = 0;    
    
    // Copy raw data to sort data array
    memcpy(sortData, data, sizeof(data));
    
    // Insertion Sort
    for(int i=1; i<=(DATASIZE-1); i++) {
      temp = sortData[i];
      j = i-1;
      while(temp<sortData[j] && j>=0) {
        sortData[j+1] = sortData[j];
        j = j-1;
      }
      sortData[j+1] = temp;
    }
    return data[(DATASIZE)>>1]; // return data value in middle of sorted array
  } 
};

////////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(m, a, b)
////////////////////////////////////////////////////////////////////////////////

float vectorDotProduct(int length, float vector1[], float vector2[])
{
  float dotProduct = 0;
  //int   i;

  for (int i = 0; i < length; i++)
  {
  dotProduct += vector1[i] * vector2[i];
  }

  return dotProduct;
}

////////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
////////////////////////////////////////////////////////////////////////////////

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

////////////////////////////////////////////////////////////////////////////////
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
////////////////////////////////////////////////////////////////////////////////

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar)
{
  //int i;

  for (int i = 0; i < length; i++)
  {
   scaledVector[i] = inputVector[i] * scalar;
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////

void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[])
{
  //int i;

  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] + vectorB[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Compute difference of 2 vectors
//  Subtract vector a from vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorSubtract(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////

void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[])
{
  //int i;

  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] - vectorB[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[])
{
  //int i, j, k;

  for (int i = 0; i < aRows * bCols; i++)
  {
    matrixC[i] = 0.0;
  }

  for (int i = 0; i < aRows; i++)
  {
    for(int j = 0; j < aCols_bRows; j++)
    {
      for(int k = 0;  k < bCols; k++)
      {
       matrixC[i * bCols + k] += matrixA[i * aCols_bRows + j] * matrixB[j * bCols + k];
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  //int i;

  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  Matrix Subtraction
//  Subtract matrix A from matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixSubtract(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  //int i;

  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] - matrixB[i];
  }
}


////////////////////////////////////////////////////////////////////////////////
//  Matrix Scaling
//  Scale matrix A, dimensions m x n, by a scaler, S
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixScale(m, n, C, S, B)
////////////////////////////////////////////////////////////////////////////////

void matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[])
{
  //int i;

  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = scaler * matrixA[i];
  }
}

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Transpose
//  Compute 3 x 3 Transpose of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Transpose3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixTranspose3x3(float matrixC[9], float matrixA[9])
{

  matrixC[0] = matrixA[0];
  matrixC[1] = matrixA[3];
  matrixC[2] = matrixA[6];
  matrixC[3] = matrixA[1];
  matrixC[4] = matrixA[4];
  matrixC[5] = matrixA[7];
  matrixC[6] = matrixA[2];
  matrixC[7] = matrixA[5];
  matrixC[8] = matrixA[8];
}

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Inverse
//  Compute 3 x 3 Inverse of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Inverse3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixInverse3x3(float matrixC[9], float matrixA[9])
{

  float det;
  float transposeA[9];
  float minors[9];
  float transposeMinors[9];

  det = matrixA[0] * (matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7]) -
        matrixA[1] * (matrixA[3] * matrixA[8] - matrixA[5] * matrixA[6]) +
        matrixA[2] * (matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6]);

  matrixTranspose3x3(transposeA, matrixA);

  minors[0] = matrixA[4] * matrixA[8] - matrixA[5] * matrixA[7];
  minors[1] = matrixA[5] * matrixA[6] - matrixA[3] * matrixA[8];
  minors[2] = matrixA[3] * matrixA[7] - matrixA[4] * matrixA[6];
  minors[3] = matrixA[2] * matrixA[7] - matrixA[1] * matrixA[8];
  minors[4] = matrixA[0] * matrixA[8] - matrixA[2] * matrixA[6];
  minors[5] = matrixA[1] * matrixA[6] - matrixA[0] * matrixA[7];
  minors[6] = matrixA[1] * matrixA[5] - matrixA[2] * matrixA[4];
  minors[7] = matrixA[2] * matrixA[3] - matrixA[0] * matrixA[5];
  minors[8] = matrixA[0] * matrixA[4] - matrixA[1] * matrixA[3];

  matrixTranspose3x3(transposeMinors, minors);

  det = 1/det;

  matrixScale(3,3, matrixC, det, transposeMinors);
}


// Alternate method to calculate arctangent from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
float arctan2(float y, float x) {
  float coeff_1 = PI/4;
  float coeff_2 = 3*coeff_1;
  float abs_y = abs(y)+1e-10;      // kludge to prevent 0/0 condition
  float r, angle;
   
  if (x >= 0) {
    r = (x - abs_y) / (x + abs_y);
    angle = coeff_1 - coeff_1 * r;
  }
  else {
    r = (x + abs_y) / (abs_y - x);
    angle = coeff_2 - coeff_1 * r;
  }
  if (y < 0)
    return(-angle);     // negate if in quad III or IV
  else
    return(angle);
}

// Used for sensor calibration
// Takes the median of 50 results as zero
#if defined(AeroQuadMega_CHR6DM) || defined(APM_OP_CHR6DM)
float findMedian(float *data, int arraySize) {
  float temp;
#else
int findMedian(int *data, int arraySize) {                  //Thanks ala42! Post: http://aeroquad.com/showthread.php?1369-The-big-enhancement-addition-to-2.0-code/page5
  int temp;
#endif
  boolean done = 0;
  //byte i;
  
   // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (byte i = 0; i<(arraySize-1); i++) {
      if (data[i] > data[i+1]) {     // numbers are out of order - swap
        temp = data[i+1];
        data[i+1] = data[i];
        data[i] = temp;
        done = 0;
      }
    }
  }
  
  return data[arraySize/2]; // return the median value
}
