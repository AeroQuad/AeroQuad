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

#define DATASIZE 25


// Low pass filter, kept as regular C function for speed
float filterSmooth(float currentData, float previousData, float smoothFactor) {
  if (smoothFactor != 1.0) //only apply time compensated filter if smoothFactor is applied
    return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor)); 
  else
    return currentData; //if smoothFactor == 1.0, do not calculate, just bypass!
}

float filterSmoothWithTime(float currentData, float previousData, float smoothFactor, float dT_scaledAroundOne) {  //time scale factor
  if (smoothFactor != 1.0) //only apply time compensated filter if smoothFactor is applied
    return (previousData * (1.0 - (smoothFactor * dT_scaledAroundOne)) + (currentData * (smoothFactor * dT_scaledAroundOne))); 
  else
    return currentData; //if smoothFactor == 1.0, do not calculate, just bypass!
}



// ***********************************************************************
// *********************** Median Filter Class ***************************
// ***********************************************************************
// Median filter currently not used, but kept if needed for the future
// To declare use: MedianFilter filterSomething;

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

//**********************************************************************************************
//
//  Vector Dot Product
//  Return the Dot product of vectors a and b with length m
//
//  Call as: vectorDotProduct(m, a, b)
//
//**********************************************************************************************
float vectorDotProduct(int length, float vector1[], float vector2[])
{
  float dotProduct = 0;
  for (int i = 0; i < length; i++) {
    dotProduct += vector1[i] * vector2[i];
  }
  return dotProduct;
}

//**********************************************************************************************
//
//  Multiply a vector by a scalar
//  Mulitply vector a with length m by a scalar
//  Place result in vector b
//
//  Call as: vectorScale(m, b, a, scalar)
//
//**********************************************************************************************
void vectorScale(int length, float scaledVector[], float inputVector[], float scalar)
{
  for (int i = 0; i < length; i++)
  {
   scaledVector[i] = inputVector[i] * scalar;
  }
}

//**********************************************************************************************
//
//  Compute sum of 2 vectors
//  Add vector a to vector b, both of length m
//  Place result in vector c
//
//  Call as: vectorAdd(m, c, b, a)
//
//**********************************************************************************************
void vectorAdd(int length, float vectorC[], float vectorA[], float vectorB[])
{
  for(int i = 0; i < length; i++)
  {
     vectorC[i] = vectorA[i] + vectorB[i];
  }
}

//**********************************************************************************************
//
//  Vector Cross Product
//  Compute the cross product of vectors a and b with length 3
//  Place result in vector C
//
//  Call as: vectorDotProduct(c, a, b)
//
//**********************************************************************************************
void vectorCrossProduct(float vectorC[3], float vectorA[3], float vectorB[3])
{
  vectorC[0] = (vectorA[1] * vectorB[2]) - (vectorA[2] * vectorB[1]);
  vectorC[1] = (vectorA[2] * vectorB[0]) - (vectorA[0] * vectorB[2]);
  vectorC[2] = (vectorA[0] * vectorB[1]) - (vectorA[1] * vectorB[0]);
}

//**********************************************************************************************
//
//  Matrix Multiply
//  Multiply matrix A times matrix B, matrix A dimension m x n, matrix B dimension n x p
//  Result placed in matrix C, dimension m x p
//
//  Call as: matrixMultiply(m, n, p, C, A, B)
//
//**********************************************************************************************
void matrixMultiply(int aRows, int aCols_bRows, int bCols, float matrixC[], float matrixA[], float matrixB[])
{
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
  
//**********************************************************************************************
//
//  Matrix Addition
//  Add matrix A to matrix B, dimensions m x n
//  Result placed in matrix C, dimension m x n
//
//  Call as: matrixAdd(m, n, C, A, B)
//
//**********************************************************************************************
void matrixAdd(int rows, int cols, float matrixC[], float matrixA[], float matrixB[])
{
  for (int i = 0; i < rows * cols; i++)
  {
    matrixC[i] = matrixA[i] + matrixB[i];
  }
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
  byte i;
  
   // Sorts numbers from lowest to highest
  while (done != 1) {        
    done = 1;
    for (i=0; i<(arraySize-1); i++) {
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
