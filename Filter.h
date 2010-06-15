/*
  AeroQuad v1.8 - June 2010
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

#ifndef __FILTER_H__
#define __FILTER_H__

/*class Filter {
// This does not work as expected
// Reverting back to proven C function call
private:
  float _smoothFactor;
  float _previousData;
  float _smoothedData;
  float newData, oldData;

public:
  Filter() {
    _smoothFactor = 1.0;
    _previousData = 0;
  }
  
  void initialize(float smoothFactor) {
    _smoothFactor = smoothFactor;
    _previousData = 0;
  }
    
  float smooth(int currentData) {
    _smoothedData = (_previousData * (1.0 - _smoothFactor)) + ((float)currentData * _smoothFactor);
    _previousData = _smoothedData;
    return _smoothedData;
  }
  
  void setSmoothFactor(float value) {_smoothFactor = value;}
  float getSmoothFactor(void) {return _smoothFactor;}
  float getData(void) {return _smoothedData;}
};*/

int smooth(int currentData, int previousData, float smoothFactor) {
  return (previousData * (1 - smoothFactor) + (currentData * smoothFactor));
}

#endif
