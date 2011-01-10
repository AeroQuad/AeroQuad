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

// Low pass filter, kept as regular C function for speed
float smooth(float currentData, float previousData, float smoothFactor) {
  if (smoothFactor != 1.0) //only apply time compensated filter if smoothFactor is applied
    return (previousData * (1.0 - smoothFactor) + (currentData * smoothFactor)); 
  else
    return currentData; //if smoothFactor == 1.0, do not calculate, just bypass!
}

float smoothWithTime(float currentData, float previousData, float smoothFactor, float dT_scaledAroundOne) {  //time scale factor
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
  #define DATASIZE 25
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
    int temp, i, j; // used to sort array

    // Insert new data into raw data array round robin style
    data[dataIndex] = newData;
    if (dataIndex < (DATASIZE-1)) dataIndex++;
    else dataIndex = 0;    
    
    // Copy raw data to sort data array
    memcpy(sortData, data, sizeof(data));
    
    // Insertion Sort
    for(i=1; i<=(DATASIZE-1); i++) {
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


