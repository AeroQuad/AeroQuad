/*
  AeroQuad v2.0.1 - September 2010
  www.AeroQuad.com
  Copyright (c) 2010 Ted Carancho.  All rights reserved.
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

// User this as a template for new classes or subclasses

// ***********************************************************************
// ************************** Example Class ******************************
// ***********************************************************************
class exampleClass {
public: 
  int exampleVariable;
  float exampleData[3];
  exampleClass(void) { 
    // this is the constructor of the object and must have the same name 
    // can be used to initialize any of the variables declared above 
  }

  // **********************************************************************
  // The following function calls must be defined inside any new subclasses
  // **********************************************************************
  virtual void initialize(void); 
  virtual void exampleFunction(int); 
  virtual const int getExampleData(byte);
  
  // *********************************************************
  // The following functions are common between all subclasses
  // *********************************************************
  void examplePublicFunction(byte axis, int value) {
    // insert common code here 
  }
  const int getPublicData(byte axis) {
    return exampleData[axis];
  }
};

// ***********************************************************************
// ************************ Example Subclass *****************************
// ***********************************************************************
class exampleSubClass : public exampleClass { 
private:
  int exampleArray[3]; // only for use inside this subclass
  int examplePrivateData; // only for use inside this subclass
  void examplePrivateFunction(int functionVariable) {
    // it’s possible to declare functions just for this subclass 
  }
  
public: 
  exampleSubClass() : exampleClass(){
    // this is the constructor of the object and must have the same name
    // can be used to initialize any of the variables declared above
  }

  // ***********************************************************
  // Define all the virtual functions declared in the main class
  // ***********************************************************
  void initialize(void) {
    // insert code here 
  }
  void exampleFunction(int someVariable) {
    // insert code here 
    examplePrivateFunction(someVariable); 
  }
  const int getExampleData(byte axis) { 
    // insert code here 
    return exampleArray[axis]; 
  } 
};
