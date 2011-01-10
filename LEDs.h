/*
  AeroQuad v2.1.3 Beta - December 2010
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

// Written by JH

// *************************************************************************
// ************************** LED Flash Class ******************************
// *************************************************************************
class LEDs {
public: 
  bool on;
protected:
  unsigned long previousTime;
  unsigned long flashTime;
public:
  LEDs(void) { 
    on = false;
  }

  virtual void initialize(void);
  virtual void run(unsigned long currentTime);
  
  bool canRun(unsigned long currentTime)
  {
    if(on)
    {
      if (currentTime > previousTime + flashTime){
        previousTime = currentTime;
        return true;
      }
    }
    return false;
  }
};

// *************************************************************************
// ************************** Flash All LEDs  ******************************
// *************************************************************************
class LEDs_FlashAll : LEDs{
private:
  int pin;
  bool state;
  bool flash;

public:
  LEDs_FlashAll() : LEDs(){}

  void initialize(void)
  {
    pin = 22;
    state = false;
    flash = false;
    
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH); //turn on the LEDs
  }
  
  void run(unsigned long currentTime)
  {
    if(canRun(currentTime))
    {
      if(flash)
      {
        digitalWrite(pin, state);
        state = !state;
      }else if(!state)
      {
        digitalWrite(pin, HIGH);
        state = true;
      }
    }
  }
  
  void off()
  {
     on = false;
     state = false;
     digitalWrite(pin, LOW);
  }

  void alwaysOn()
  {
    on = true;
    flash = false;
    flashTime = 0; 
  }
  
  void flashSlow()
  {
    on = true;
    flash = true;
    flashTime = 500000; 
  }
  
  void flashFast()
  {
    on = true;
    flash = true;
    flashTime = 100000; 
  }
};


