/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
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

#ifndef _AQ_MATH_H_
#define _AQ_MATH_H_

#define DATASIZE 25

#include "Arduino.h"

#define G_2_MPS2(g) ((g) * 9.80665)
#define MPS2_2_G(m) ((m) * 0.10197162)

// Low pass filter, kept as regular C function for speed
float filterSmooth(float currentData, float previousData, float smoothFactor);
float filterSmoothWithTime(float currentData, float previousData, float smoothFactor, float dT_scaledAroundOne);

// ***********************************************************************
// *********************** Median Filter Class ***************************
// ***********************************************************************
// Median filter currently not used, but kept if needed for the future
// To declare use: MedianFilter filterSomething;

class MedianFilter 
{
public: 
  float data[DATASIZE], sortData[DATASIZE];
  int dataIndex;
  MedianFilter();

  void initialize();
  
  const float filter(float newData);
};

////////////////////////////////////////////////////////////////////////////////
//  Vector Dot Product
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(m, a, b)
////////////////////////////////////////////////////////////////////////////////

float vectorDotProduct(int length, float vector1[], float vector2[]);

////////////////////////////////////////////////////////////////////////////////
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
////////////////////////////////////////////////////////////////////////////////

void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3]);

////////////////////////////////////////////////////////////////////////////////
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
////////////////////////////////////////////////////////////////////////////////

void vectorScale(int length, float scaledVector[], float inputVector[], float scalar);

////////////////////////////////////////////////////////////////////////////////
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////

void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[]);

////////////////////////////////////////////////////////////////////////////////
//  Compute difference of 2 vectors
//  Subtract vector a from vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorSubtract(m, c, b, a)
////////////////////////////////////////////////////////////////////////////////

void vectorSubtract(int length, float vectorC[], float vectorA[], float vectorB[]);

////////////////////////////////////////////////////////////////////////////////
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[]);

////////////////////////////////////////////////////////////////////////////////
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);

////////////////////////////////////////////////////////////////////////////////
//  Matrix Subtraction
//  Subtract matrix A from matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixSubtract(m, n, C, A, B)
////////////////////////////////////////////////////////////////////////////////

void matrixSubtract(int rows, int cols, float matrixC[], float matrixA[], float matrixB[]);


////////////////////////////////////////////////////////////////////////////////
//  Matrix Scaling
//  Scale matrix A, dimensions m x n, by a scaler, S
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixScale(m, n, C, S, B)
////////////////////////////////////////////////////////////////////////////////

void matrixScale(int rows, int cols, float matrixC[], float scaler, float matrixA[]);

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Transpose
//  Compute 3 x 3 Transpose of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Transpose3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixTranspose3x3(float matrixC[9], float matrixA[9]);

////////////////////////////////////////////////////////////////////////////////
//  3 x 3 Matrix Inverse
//  Compute 3 x 3 Inverse of A
//  Result placed in matrix C, dimension 3 x 3
//
//  Call as: Inverse3x3(C, A)
////////////////////////////////////////////////////////////////////////////////

void matrixInverse3x3(float matrixC[9], float matrixA[9]);


// Alternate method to calculate arctangent from: http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
float arctan2(float y, float x);

// Used for sensor calibration
// Takes the median of 50 results as zero
float findMedianFloat(float *data, int arraySize);
int findMedianInt(int *data, int arraySize); 

boolean isSwitched(float previousError, float currentError);

float invSqrt(float x);
int32_t  isq(int32_t  x);

#endif
