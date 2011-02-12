class MultipilotAccelerometer : public Accelerometer 
{
public:

  MultipilotAccelerometer() : Accelerometer()
  {
    // Accelerometer Values
    // Update these variables if using a different accel
    // Output is ratiometric for ADXL 335
    // Note: Vs is not AREF voltage
    // If Vs = 3.6V, then output sensitivity is 360mV/g
    // If Vs = 2V, then it's 195 mV/g
    // Then if Vs = 3.3V, then it's 329.062 mV/g
    // Accelerometer Values for LIS344ALH set fs to +- 2G
    // Vdd = 3.3 Volt
    // Zero = Vdd / 2
    // 3300 mV / 5  (+-2G ) = 660
    accelScaleFactor = 0.000660;
  }
  
  void initialize() 
  {
    // rollChannel = 6
    // pitchChannel = 7
    // zAxisChannel = 5
    this->_initialize(6, 7, 5);
  }
  
  void measure() 
  {
    currentTime = micros();
    for (byte axis = ROLL; axis < LASTAXIS; axis++) 
    {
      accelADC[axis] = analogRead(accelChannel[axis]) - accelZero[axis];
      accelData[axis] = filterSmoothWithTime(accelADC[axis], accelData[axis], smoothFactor, ((currentTime - previousTime) / 5000.0));
    }
    previousTime = currentTime;
  }
  
  const int getFlightData(byte axis) 
  {
    return getRaw(axis);
  }
  
  // Allows user to zero accelerometers on command
  void calibrate() 
  {
    int findZero[FINDZERO];
    for (byte calAxis = ROLL; calAxis < LASTAXIS; calAxis++) 
    {
      for (int i=0; i<FINDZERO; i++)
      {
        findZero[i] = analogRead(accelChannel[calAxis]);
      }
      accelZero[calAxis] = findMedian(findZero, FINDZERO);
    }

    // store accel value that represents 1g
    accelOneG = accelZero[ZAXIS];
    // replace with estimated Z axis 0g value
    accelZero[ZAXIS] = (accelZero[ROLL] + accelZero[PITCH]) / 2;
  }

/*  void calculateAltitude() 
  {
    currentTime = micros();
    if ((abs(getRaw(ROLL)) < 1500) && (abs(getRaw(PITCH)) < 1500))
    { 
      rawAltitude += (getZaxis()) * ((currentTime - previousTime) / 1000000.0);
    }
    previousTime = currentTime;
  } 
*/  
};