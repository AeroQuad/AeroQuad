AeroQuad Flight Software Source Code 3.1 Beta 1
========================================
[http://www.AeroQuad.com](http://www.AeroQuad.com)

Version 3.1 Beta 1 Release Notes (7/1/2012)
----------------------------------------
* Waypoint following (Waypoints uploaded from Configurator)
* Auto Descent/Land (Must be tuned for your individual setup)
* GPS Position Hold
* GPS Return to Home
* Obstacle detection
 * Uses ultrasonic sensors to detect if objects are nearby and displays on the OSD
* Telemetry over audio channel of your video transmitter
 * Requires telemetry modem soon to come to AeroQuad store)
* Option to enable 400Hz update rate to motors (ESC's must support this)
* Fixed hex configuration motor control
* OSD improvements to support above features
 * Ability to tune PIDs and store them
 * Graphical indication when objects are nearby
 * Icons to denote flight mode (acro, stable, position hold)
 * Return to home arrow and distance to home
* Transmitter switch assignments for new features
 * AUX1 or AUX2 < 1750 = Altitude hold (baro or ultrasonic)
 * AUX3 < 1750 = Auto descent/land (baro or ultrasonic)
* If GPS Navigation Enabled
 * AUX2 < 1750 = Start waypoint following
 * AUX1 < 1600 = GPS position hold
  * If AUX1 in middle switch position = altitude hold
  * If AUX1 in last position = GPS position hold
* Support for AeroQuad 32 board
 * Updated Maple libraries

Version 3.0.1 Release Notes (2/21/2012)
----------------------------------------
* Updated min/max motor commanding for level flight during yaw
* Increased floating point precision used for transmitter calibration
* Removed motor spin up when arming/disarming motors
* Improved battery monitor & OSD

Version 3.0 Release Notes (1/29/2012)
----------------------------------------
* Multiple flight configurations are supported
 * Quad X
 * Quad +
 * Quad Y4
 * Tri
 * Hex X
 * Hex +
 * Hex Y6
 * Octo X
 * Octo +
 * Octo X8
* Multiple flight angle estimation algorithms supported
 * Improved DCM (best with magnetometer)
 * ARG (best with no magnetometer)
 * MARG (experimental)
* Flight options supported
 * Heading hold with magnetometer or gyro
 * Altitude hold with barometer
 * Altitude hold with ultrasonic sensor (best for low altitude hold and terrain following)
* Enhanced battery monitoring options
 * Enable auto descent
 * Specify battery cell count
 * Integration with On Screen Display (OSD)
* Multiple receiver options
 * 6 or 8 channel receivers supported
 * PWM receivers
 * PPM receivers
 * PPM using hardware timer
  * Specific support for Spektrum, Graupner, Robe, Hitec, Futaba, Hitec, Sanwa & others
* Telemetry options
 * Wireless telemetry on dedicated serial port
 * OpenLog binary write
* Camera stabilization support
 * Dedicated servo channels for roll, pitch, yaw
* Custom OSD support for MAX7456
 * Specify video standard to use
 * Specify callsign to display
 * Built in attitude indicator
 * Display altitude in feet/meters
 * OSD system which allows remote PID tuning!  

Version 2.5.1 Release Notes (12/22/2011)
----------------------------------------
* Supports the new Arduino 1.0 IDE
* Fixed magnetometer issue where mag output intermittently output zero's
* Fixed calibration bug where X & Y axis were swapped
* Added support for hexa configurations
 * Look at the file Motors.h to view how to connect the motors
* This version is only compatible with the AeroQuad v2.7.1 Configurator
 * Please note support for > 4 motors not directly supported within v2.7.1 Configurator
 * To view motor data, use Serial Monitor to send '=' command
* Special thanks to Jihlein for producing this version and to ala42 for his bug fixes!

Version 2.5 Release Notes (11/30/2011)
----------------------------------------
* Added support for the new v2.1 AeroQuad Shield with HMC5883L
* Added support for mounting an HMC5883L upside down on v2.0 AeroQuad Shield
* Added support for octo configurations (thanks Jihlein!)
* This version is only compatible with the AeroQuad v2.7.1 Configurator

 Version 2.5 Beta 1 Release Notes (7/31/2011)
----------------------------------------
 * Fixed various 8 channel transmitter channel bugs (special thanks to Ala42!)
 * Added safety procedure for ESC calibration (prevent random full throttles during PID updates)
   * Please keep common sense safety practices in place, remove props or remove battery power when necessary
 * Optmized SerialCom.pde (removed unused commands)
 
Version 2.4.3 Release Notes (7/24/2011)
----------------------------------------
 * Fixed duplicate altitude hold call in timing executive
 * Added support for 8 channel receiver for Mega (thanks Moeffe)
 * Added telemetry response to tell Configurator how many receiver channels and motor commands are available
 
Version 2.4.2 Release Notes (6/29/2011)
----------------------------------------
 * Fixed EEPROM issue that affects altitude hold (thanks Aadamson)
 * Greatly improved altitude hold algorithm (thanks Aadamson)
 * Updated incorrect sign issue with v1.7 shield
 * Fixed Issue 114:	Processor specific Motors_PWMtimer class is not surrounded by ifdef

Version 2.4.1 Release Notes (6/5/2011)
----------------------------------------
 * Wii bug fixes (thanks aadamson/jihlein) - please note that the Wii sensor orientations now follow the MultiWii convention
 * Video On Screen Display support (thanks Alamo)
 * CHR6DM compilation bug fixes (thanks lokling)

Version 2.4 Release Notes (4/2/2011)
----------------------------------------
 * Added ARG/MARG flight angle estimation routines
 * Improved timing executive, reduced main flight loop to 100Hz
 * Stable/Attitude Mode has highly improved auto level capability
 * All these changes have been made by jihlein and aadamson, THANKS!
 
 Version 2.3 Release Notes (3/17/2011)
----------------------------------------
 * Implemented common SI units for sensors
 * Implemented common DCM gains for all hardware platforms
 * Calculate heading using Compass & DCM
 * Acro Mode utilizes SI units only for common starting gains compared with other hardware platorms
 * New altitude hold updates
   * initialized PID with hold altitude as last postion
   * created a deadband in the throttle stick when in altitude hold of 250 step (configurable in aeroquad.h), this also is the extents of the PANIC alarm as noted below.
   * Added a PANIC mode, where if you move the throttle greater than 250 steps, it will automatically turn off Altitude hold and the throttle will function as it normally does, to turn back on altitude hold, you have to turn it off and back on again, allow for a get out of altitude hold without having to find the switch.  This PANIC can either be UP or DOWN throttle.
   * Added a bump up or down to altitude hold. Once you have Altitude hold enabled, and you are in the throttle dead band, you can nudge the altitude up or down, by moving the throttle slightly up or slightly down and then back to the middle.  Scaled the climb or descent to be really slow.  Throttle increase is based upon altitude increase and constrained to no more than +/- 50 units.

Version 2.2 Release Notes (2/6/2011)
----------------------------------------

  * Fixed correct scaling for Kd for all flight modes (thanks to Ziojos & Honk).  Makes it easier to find the right PID gains to remove oscillations during flight.  PID Tuning Guide is updated with best values to use.
  * Added experimental ArduPirate Stable Mode (thanks to Kenny9999)
  * Started common SI units for major sensors to work with DCM (thanks to Kenny9999)
  * Addressed issues 82-100 in Google Code Issue Tracker (http://code.google.com/p/aeroquad/issues/list)

Version 2.1 Release Notes (1/20/2011)
----------------------------------------

  * Added support for battery monitor for AeroQuad v2.0 Shield
  * Initialization bug fix for receiver code
  * Fixed issues 64-81 in Google Code Issue Tracker (http://code.google.com/p/aeroquad/issues/list)

Version 2.1.2 Beta Release Notes (12/22/2010)
----------------------------------------

  * Fixed PWM timer class to work with Uno, special thanks to CupOfTea (implementation) and Ala42 (debugging)!
  * Implemented PWM timer class for Mega (thanks CupOfTea!)

Version 2.1.1 Beta Release Notes (12/20/2010)
----------------------------------------

  * Fixed issues 55-63 in Google Code Issue Tracker (http://code.google.com/p/aeroquad/issues/list)
  * Added support for DCM with Uno and v1.8 Shield for improved Stable Mode performance, special thanks to Ala42 and Aadamson for all their optimization work
  * Added experimental camera stabilization code (thanks CupOfTea)

Version 2.1 Beta Release Notes (12/4/2010)
----------------------------------------

  * Has only been flight tested with Arduino Mega 2560 using v2.0 Shield
  * Bench tested configurations: Uno with v1.8 Shield, Duemilanove with v1.8 Shield, Duemilanove with v1.7 Shield
  * This has many changes and optimization improvements implemented, please be careful flying with this version as this is still a work in progress
  * Full documentation on all changes will be provided at full release
  * Many improvements by Lokling and Honk

Version 2.0.1 Release Notes (9/19/2010)
----------------------------------------

This is a bug fix release which resolves reported issues:

  * [#30](http://code.google.com/p/aeroquad/issues/detail?id=30&can=1) added serial com commands to report hardware platform and motor configuration to adjust scales
  * [#31](http://code.google.com/p/aeroquad/issues/detail?id=31&can=1) by adding serial command to report motor config, can now label Configurator correctly
  * [#32](http://code.google.com/p/aeroquad/issues/detail?id=32&can=1) fixed wrong referenced array element

Additionally fixed incorrect sensor orientation for v1.7 Shield with Duemilanove, and Wii for Duemilanove and Mega, and APM with APM sensor shield.  Added #defines for Wii for Mega.  Solved DCM overflow issues for APM when performing calibration or writing to EEPROM.  Flight tested ArduCopter hardware, v2.0 Shield with Mega, v1.8 Shield with Duemilanove, v1.7 Shield with Duemilanove, v1.8 Shield using Wii sensors with Duemilanove.  Updated default PID values for v2.0 shield using 30.5cm motor to motor configuration at 1.2kg. 

Version 2.0 Release Notes (9/6/2010)
----------------------------------------

This is a major architecture change from the v1.x flight software release.  Major hardware and algorithm components are now implemented as C++ classes to allow common flight software support.  Improved Stable Mode implemented.  Hardware support for new v1.8 and v2.0 AeroQuad Shields using ITG-3200 gyros and BMA-180 accelerometers.  Hardware support for Wii sensors, ArduCopter (APM and Oilpan) and Multipilot (all new platforms still need flight testing).

Flight software configuration support for multiple hardware has changed.  Look for the lines listed below at the start of AeroQuad.pde and uncomment the appropriate selection:

    /****************************************************************************
     ************************* Hardware Configuration ***************************
     ****************************************************************************/
    // Select which hardware you wish to use with the AeroQuad Flight Software
    
    //#define AeroQuad_v1         // Arduino 2009 with AeroQuad Shield v1.7 and below
    #define AeroQuad_v18        // Arduino 2009 with AeroQuad Shield v1.8
    //#define AeroQuad_Wii        // Arduino 2009 with Wii Sensors
    //#define AeroQuadMega_v1     // Arduino Mega with AeroQuad Shield v1.7 and below
    //#define AeroQuadMega_v2     // Arduino Mega with AeroQuad Shield v2.x
    //#define AeroQuadMega_Wii    // Arduino Mega with Wii Sensors (needs debug)
    //#define ArduCopter          // ArduPilot Mega (APM) with APM Sensor Board
    //#define Multipilot          // Multipilot board with Lys344 and ADXL 610 Gyro
    //#define MultipilotI2C       // Active Multipilot I2C and Mixertable

Review the rest of the #defines also to match the unique setup of you multicopter.

Version 1.7.1 Release Notes (3/24/2010)
----------------------------------------

Fixed bug for Arduino Mega users.  #define Mega_AQ1x was not defined before #include "Receiver.h".  It accidentally caused the receiver pin assignments to use the Duemilanove assignments instead of the Mega.

Version 1.7 Release Notes (3/21/2010)
----------------------------------------

This release allows users to specific which voltage is used for aref, specifically for compatibility with AeroQuad v1.7 shields.  Updated accelerometer calibration [here][1] to accommodate user definable aref and to best estimate Z axis zero position.  Fixed heading hold reference to aref and optimized motor to gyro rate conversion for motor control PID.  The new aref update is configured through the AeroQuad Configurator v2.3.  Tested this version against Arduino 0018.  Be sure to update your HardwareSerial.cpp found [here][2] if you are using XBee for wireless communication.

Version 1.6 Release Notes (3/5/2010)
----------------------------------------

This release fixes a yaw bug that exhibits itself with the new capacitors installed which allows the user to use higher PID values.  Also the comments were fixed to reflect usage of IDG500 or IXZ500 gyros.  If the user does not select the correct gyro, the yaw axis may become inverted.  Started implementing certain functions using classes (C++).  The FlightAngle class defines the algorithm to use for angle estimation, the Motors class defines how PWM works and the Filter class allows multiple filter objects to be called to reduce the number of global variables needed and to encapsulate and retain the data needed for those filters to work.

Version 1.5 Release Notes
----------------------------------------

This is a maintenance release for users of an Arduino Mega with an AeroQuad Shield v1.5 which provides receiver support.  There is a bug in the Arduino core code which doesn't allow the proper PCINT assignments to PCINT 8-23.  This release will hardcode AI pins 8-13 (PCINT 16-21) for use as receiver pins.  To enable this capability, please uncomment #define Mega_AQ1x located in AeroQuad.pde.

Place jumper wires as indicated below to make your AeroQuad Shield v1.5 receiver pins work with an Arduino Mega:

  * Roll (Aileron) Channel, place jumper between AQ Shield pin 2 and Mega AI13 
  * Pitch (Elevator) Channel, place jumper between AQ Shield pin 5 and Mega AI11
  * Yaw (Rudder) Channel, place jumper between AQ Shield pin 6 and Mega AI10
  * Throttle Channel, place jumper between AQ Shield pin 4 and Mega AI12
  * Mode (Gear) Channel, place jumper between AQ Shield pin 7 and Mega AI9
  * Aux Channel, place jumper between AQ Shield 8 and Mega AI8

Version 1.4 Release Notes
----------------------------------------

Warning!  If you are a previous AeroQuad user and are using the Sparkfun 5DOF with the original IDG300 gyros, please review AeroQuad.pde and uncomment the line: #define OriginalIMU.  Failure to do so will result in unstable flight.  When Sparkfun updated the 5DOF IMU to use the IDG500, the roll/pitch gyro axes were inverted.  The default AeroQuad Flight Software behavior is to assume the user is using the latest Sparkfun 5DOF IMU to make it easier for new users.

Many of the improvements in v1.4 are geared towards the new features of the AeroQuad Configurator v2.0 and include the Spectrum Analyzer, Serial Monitor, manual input motor commands for initial checkout.  A basic heading hold has also been implemented (turned off by default) using the yaw gyro.

The camera stabilization feature is operational but still not optimal.  There is an incompatibility with PCINT for reading receiver output and the Arduino 0017 Servo library.  There is an intermittent jitter that occurs during reading of receiver output.  Using analogWrite() works well, but there are not enough compatible pins on the Arduino Duemilanove to support this.  Therefore further development will be pushed to the Arduino Mega.

The heading hold feature is also operational but not optimal.  The IDG gyros exhibit some drift during flight, which will cause the heading hold to eventually also drift.  This can be alleviated by landing the AeroQuad, disarm motor output, and then perform a manual sensor calibration (TX left stick to the lower left, TX right stick to the lower right).  The user will then be able to return to flight.  This feature will be optimized during Arduino Mega development since it is planned to implement heading hold with a magnetometer for that platform.

With the updated Servo library of Arduino 0017, the older ServoTimer2 library will not be used for future versions of the AeroQuad Flight Software.

Version 1.3.2 Release Notes
----------------------------------------

This version provides additional communication messages for the Configurator to make the new calibration procedures more robust.  New support added to allow Configurator to auto-reconnect to the AeroQuad (via USB or Wireless) for more convenience to the user. The Pin Change Interrupt (PCINT) code has been improved to work better for Futaba transmitters.  The variable declaration section in the main AeroQuad.pde sketch has been organized into separate header files for easier maintainability into the future.  The new default EEPROM values now include a suggested setting for yaw to allow a smoother yaw transition (in the past it would cause the quad to pop up and down a bit).  The main loop is now organized into different timed loops, to make sure the sampling of the sensors and control algorithm execution are now performed at regular timed intervals (500Hz). 
 
If you'd like to maintain as much PWM resolution as possible for PWM Motor Control, please install the ServoTimer2 library found at:
[http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1204020386/75][3].  To compile the AeroQuad sketch to use the ServoTimer2 library, the ServoTimer2 folder found in this distribution must be copied to ..\Arduino-00xx\hardware\libraries.  To verify the library has been successfully installed, open the Arduino IDE and go to Sketch->Import Library. You should see ServoTimer2 as one of the items to select (you don't need to select it at this time).  The last step is to configure the #define statements for ServoTimer2 in the AeroQuad.pde sketch.

The default #define statement for PWM Motor Control is AnalogWrite.  This will provide an update rate to the motors at 490Hz.  No modification to the code, or installation of the ServoTimer2 library is necessary for this default.

Version 1.3.1 Release Notes
----------------------------------------

This version allows the user to calibrate a transmitter in either Airplane or Helicopter Mode.  There is also a calibration for ESC's and the ability for the user to return EEPROM values to a default value (which is also useful for first time setup).  To use these features, you must download the AeroQuad Configurator v1.3.1 or greater.  The transmitter calibration is meant to fix the situation where the trims could potentially move the transmitter beyond the 1000-2000 ms PWM pulse width range the AeroQuad is expecting.  This had resulted in a non-response from the AeroQuad when moving the transmitter stick to an extreme position.

Version 1.2 Release Notes
----------------------------------------

This version incorporates the use of the Pin Change Interrupts (PCINT) to read output from an R/C receiver.  This removes the need to know the channel order of the receiver used.  This PCINT method has been tested with:

  * Spektrum DX7 w/ AR6100, 6200 and AR7000
  * Futaba T6EXHP w/ R146iP
  * Airtronics RD8000 w/ 92778

The PCINT pins are unfortunately different between the Arduino Duemilanove and Arduino Mega.  Therefore Version 1.2 is not directly compatible with the Mega.  There are plans for compatibility with future releases, but for now the Mega is only compatible with Version 1.0 of the AeroQuad flight software.

**New v1.2 features:**

  * Stable Mode (auto level) is disabled by default.  To enable remove the comment of the appropriate #define statement in AeroQuad.pde.  This was done to remove prevent confusion by new users.
  * If any critical flight parameters are zero, it is automatically filled in with a typical value.
  * PCINT receiver code updated for efficiency
  * Auto calibration of sensors at powerup disabled by default.  To enable, remove the comment of the appropriate #define statement in AeroQuad.pde.
  * Manual calibration of sensors can be performed by moving left transmitter stick to the lower left, and the right transmitter stick to the lower right corners.

Version 1.1 Release Notes
----------------------------------------

This version of the code now uses analogWrite() to efficiently write PWM commands to the ESC's.  The tradeoff is that we can only achieve 128 steps of resolution.  The Turnigy ESC's specified in the parts list have been measured to only have 128 steps of resolution, so if you are using this ESC, there shouldn't be any issues.  Also, if you've built a previous MikroQuad or AeroQuad you will be required to update wiring in your shield per the AeroQuad website instructions.

**New v1.1 features:**

  * 400Hz update rate to ESC's/Motors
  * Combined user configurable values into single tab in Configurator

Happy flying!  
[info@AeroQuad.com](mailto:info@AeroQuad.com)


  [1]: http://carancho.com/AeroQuad/forum/index.php?topic=290.msg2735#msg2735
  [2]: http://carancho.com/AeroQuad/forum/index.php?topic=85.0
  [3]: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1204020386/75