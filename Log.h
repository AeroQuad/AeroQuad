/*

 Author: Bill Faulkner
 
 For use with 
 AeroQuad v2.4 - April 2011
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

#ifndef LOG_H
#define LOG_H

//  This class is used with the OpenLog serial logger from Sparkfun.
//  Data is sent in raw binary form (not ASCII) for speed. A header
//  is written first that describes the structure of each type of
//  log record.
//
//  Basic Structure each record
//
//  Start  Start byte
//  Type   Record Type byte
//  Data   Variable length data
//  Cksum  Checksum byte
//


#define  LOG_OL_PROMPT         '>'
#define  LOG_OL_ESCSEQ         "\x1A\x1A\x1A\r"    // 3 ctrl-Z and a CR
#define  LOG_OL_RESTART        "\rrestart\r"
#define  LOG_CMDESC            '~'
#define  LOG_CMDRETRIES        5
#define  LOG_RECORD_SS         0xA3

#define  LOG_REC_HEADER        0
#define  LOG_REC_FLIGHT        1  
#define  LOG_REC_GPS           2
#define  LOG_REC_ITOD          3
#define  LOG_REC_ALTHOLD       4
#define  LOG_REC_VRSWITCH      5
//#define  LOG_REC_ALTPID        6
//#define  LOG_REC_BARODATA      7
//#define  LOG_REC_BAROGND       8
#define  LOG_REC_PQR           6
#define  LOG_REC_XYZ           7
#define  LOG_REC_HDG           8
#define  LOG_REC_PT            9



////////////////////////////////////////////////////////////////////////////////
//
//  The logged output is in a binary format.  A key table indiced by log record type
//  is used by the decorder to understand how to unpack the record.  The desc
//  element of the key contains a string of letters describing the order and type
//  of data elements packed into the record.  The decorder uses these letters to
//  know how big each data element is and how to decode it.  The defined types are:
//
//  T  time, normally a long, the high order byte is chopped & the decoder handles wraparound.
//  l
//  b
//  l
//  f
//  
////////////////////////////////////////////////////////////////////////////////
#define  LOG_REC_NUMTYPES       sizeof(_descriptors)/sizeof(_descriptors[0])
struct loggerTypes {
  uint8_t      type;
  uint8_t      desc[21];  // null terminated String of decode values, max 20
  uint8_t      labels[100];
} 
_descriptors[] = {
  {LOG_REC_HEADER,  "X",        "X"},                     
  {LOG_REC_FLIGHT,  "Thhaiib", "raw Alt,sm Alt,gndZacc,Thr Adj,Mtr Cmd,AH Flg" }, 
  {LOG_REC_GPS,     "Tgghb",    "lat,long,GPS alt,flag"},
  {LOG_REC_ITOD,    "Tffffa",      "raw Alt, Roll, Pitch, Yaw"},
  {LOG_REC_ALTHOLD, "Thib",       "H Alt,H Thr,AltH f"},
  {LOG_REC_VRSWITCH,"Tv",        "bit flags"},
//  {LOG_REC_ALTPID,  "Thhii",    "H Alt,C Alt,Th Adj,Mtr Cmd"},
//  {LOG_REC_BARODATA,"Tblf",     "flags,iir_b5,iir_Alt"},
//  {LOG_REC_BAROGND, "Tfff",      "GndTemp,GndPress,GndAlt"},
  {LOG_REC_PQR,     "Tfff",        "p,q,r"},
  {LOG_REC_XYZ,     "Tfff",        "ax,ay,az"},
  {LOG_REC_HDG,     "Tfff",        "1G,hdgX,hdgY"},
  {LOG_REC_PT,      "Tff",        "Phi,Theta"}
};

#define  OPENLOG_PIN  32

class serialLogger : public Print {
  
private:
  uint8_t                    _port;
  uint8_t                    _baud;
  uint8_t                    _buffer[250];
  uint8_t                    _inCommandMode;
  HardwareSerial             *loggerSerialPort; 

public:
  using Print::write;  // pull in write(str) and write(buf, size) from Print


  //
  // Constructor
  //
  serialLogger (void) { 

    // prime the logging buffer with the start sequence
    _buffer[0] = LOG_RECORD_SS;

  } //end serialLogger()



  // 
  // Note the lack of error checking
  // 
  void begin(uint8_t port, long baud) {

    _inCommandMode = OFF;

    switch (port) {
      
      case 0: loggerSerialPort = &Serial; break;
    
#if defined(AeroQuadMega_v2) || defined(AeroQuadMega_Wii) || defined(AeroQuadMega_CHR6DM)
      case 1: loggerSerialPort = &Serial1; break;
      case 2: loggerSerialPort = &Serial2; break;
      case 3: loggerSerialPort = &Serial3; break;
#endif

      default: loggerSerialPort = &Serial; break;
    
    }

    // OpenLog will open and read the file CONFIG.TXT which should contain:
    // "115200,26,3,0", Baud=115200, Esc char is 26=0x1A, Num esc chars=3, 0 is new file mode
    // 
    // Since the quad could have been reset without the OpenLog being restarted
    // make sure we are starting with a fresh file.  
    //
    // FUTURE: Hook up OpenLog's DTR pin to an Output PIN so this code can reset OpenLog

    loggerSerialPort -> begin(baud);
    
    pinMode (OPENLOG_PIN, OUTPUT);
    digitalWrite (OPENLOG_PIN, HIGH);
    delay(22);
    

    exitCommandMode();

    //loggerSerialPort -> write(LOG_OL_RESTART);

    //delay(500);   // give logger time to restart
    //loggerSerialPort -> flush();  // get rid of echo chars.

    // write out a header record
    //dumpRecord(LOG_REC_HEADER);

  } //end begin()


  //
  //  
  //
  void write (uint8_t c) {
    loggerSerialPort -> write(c);
  }



  //
  //
  //
  //
  void dumpRecord(uint8_t type) {
    uint32_t   cksum=0;
    uint8_t    cksum_b;
    uint16_t         len=0;
    uint8_t    *bptr = &_buffer[1];
    uint8_t    *iptr = &_buffer[0];
    uint8_t    *vptr;
    uint32_t   ival;
    uint16_t   i_val;
    float      f_val;
    uint8_t    *dptr;
    float fval;
    byte  b_val;

    // if in interactive command mode, dont send records to the log
    if (_inCommandMode == ON) return;

    // built up a string & write it all at once to save function call overhead
    // nested if stmts take less space - deal with that late

//FIX: have log_rec_header write to buffer & let action at bottom calc checksum & write to OpenLog
//FIX: move cksum calc to one loop at bottom.    
    switch (type) {

      case (LOG_REC_HEADER):          //header descriptor record
      
        // start byte
        loggerSerialPort -> write(_buffer,1);
        cksum = LOG_RECORD_SS;

        // type
        loggerSerialPort -> write(type);

        // count up the lengths of the strings
        for (uint8_t i=0; i < LOG_REC_NUMTYPES; i++) 
          len+= 1+ strlen((const char *)_descriptors[i].desc)
                 + strlen((const char *)_descriptors[i].labels) +2;  // 1+ for type byte, +2 for \0 string term.

        // length of LOG_REC_HEADER type record
        loggerSerialPort -> write((uint8_t *)&len,2);
        cksum += len;

        // data - write out each descriptor record
        for (uint8_t i=0; i < LOG_REC_NUMTYPES; i++) {
          // type
          loggerSerialPort -> write(_descriptors[i].type);
          cksum += _descriptors[i].type;
          
          // desc string
          for(dptr = _descriptors[i].desc; *dptr; dptr++) cksum += *dptr;
          loggerSerialPort -> write(_descriptors[i].desc, dptr - _descriptors[i].desc);
          loggerSerialPort -> write((uint8_t) 0);  //String terminator;
          
          // labels string
          for(dptr = _descriptors[i].labels; *dptr; dptr++) cksum += *dptr;
          loggerSerialPort -> write(_descriptors[i].labels, dptr - _descriptors[i].labels);
          loggerSerialPort -> write((uint8_t) 0);  //String terminator;
        }
        
        // finally write chksum (truncated to one byte)
        cksum_b = cksum;  // not sure if we could trust a cast here.  Test later
        loggerSerialPort -> write(cksum_b);
 
      break; // end case 0: 

#define DUMP(X) vptr=(uint8_t *)&X;for(uint8_t i=0;i<sizeof(X);i++)*bptr++=*vptr++; 
#define DUMPA(X,Y) vptr=(uint8_t *)X;for(uint8_t i=0;i<Y;i++)*bptr++=*vptr++;  //addr, size 

      //  TYPE1  start with altitude stuf
/*
      case (LOG_REC_FLIGHT):
        // Type, Timestamp, 
        DUMP(type);                               // Type
        DUMPA(&currentTime,3);              //T Time   switch to delta w/ count - save 2 bytes
        i_val=altitude.rawAltitude*100;
        DUMP(i_val); //h
        i_val=altitude.altitude*100;
        DUMP(i_val); //h
        //i_val=altitude.groundAltitude*100;
        //DUMP(i_val); //h
        //DUMP(altitude.rawAltitude);           //f Raw altitude (no smoothing) : pack to int i=f*10;  save 2 bytes 
        //DUMP(altitude.altitude);              //f Smoothed altitude             pack to 2 bytes
        i_val = 0; // AKA i_val = (tempFlightAngle.groundZaccel * 1000);
        DUMP(i_val); //a
        //DUMP(flightAngle.groundZaccel);       //f Z vectored accel
  

        DUMP(throttleAdjust);                 //i Throttle adjustments for alt hold 
        i_val = motors.getMotorCommand(FRONT);
        DUMP(i_val);                          //i Motor cmd - assuming stable level hover so only dump one
        DUMP(altitudeHold);
  

         //DUMP(accel.accelData[0]);         //i X axis smooth
         //DUMP(accel.accelData[1]);         //i Y axis smooth
         //DUMP(accel.accelData[2]);         //i Z axis smooth

        //DUMP(batteryMonitor.batteryVoltage); //
        //DUMP(GPS.Altitude);               //l GPS altitude
        //DUMP(GPS.Fix);                    //b GPS Fix = 1, nofix = 0;
      break;  // end case 1
*/
/* AKA
      case (LOG_REC_GPS): 
      {
        uint8_t  gpsData;
        DUMP(type);
        DUMPA(&currentTime,3);              //T
        //DUMP(GPS.Time);                       //l Time in ms from start of week
        DUMP(GPS.Lattitude);                  //l Lattitude in degrees
        DUMP(GPS.Longitude);                  //l Longitude in degrees
        DUMPA(&GPS.Altitude,2);               //i Altitude in cm - dont need 4 bytes
        gpsData=(GPS.NumSats) | (GPS.Fix << 7);
        DUMP(gpsData);                        //b Flag indicating GPS had a fix & now many Sats     
      }              
      // throttle, motor out battery 
      break;
    
      case (LOG_REC_ITOD):
        DUMP(type);
        DUMPA(&currentTime,3); 
        DUMP(altitude.rawAltitude); //f
        f_val = tempFlightAngle.getData(ROLL);
        DUMP(f_val); //f
        f_val = tempFlightAngle.getData(PITCH);
        DUMP(f_val); //f
        f_val = tempFlightAngle.getData(YAW);
        DUMP(f_val); //f
        i_val = 0; // AKA i_val = (tempFlightAngle.groundZaccel * 1000);
        DUMP(i_val); //a
            break;
*/              
      case (LOG_REC_ALTHOLD):
        DUMP(type);
        DUMPA(&currentTime,3); 
    
        i_val=holdAltitude*100;
        DUMP(i_val); //h
        //DUMP(holdAltitude);                   //f Altitude to hold
        DUMP(holdThrottle);                   //i Throttle value used for holding - overload with AltHold flag
        DUMP(altitudeHold); 
            break;
/*            
      case (LOG_REC_VRSWITCH):
        DUMP(type);
        DUMPA(&currentTime,3); 
    
        b_val=auxSwitch.getState();
        b_val |= flightMode << 5;            
        DUMP(b_val); //h
            break;
           
      case (LOG_REC_ALTPID):
        DUMP(type);
        DUMPA(&currentTime,3);
        i_val=holdAltitude*100;
        DUMP(i_val); //h 
        i_val=altitude.getData()*100;
        DUMP(i_val);
        DUMP(throttleAdjust);                 //i Throttle adjustments for alt hold 
        i_val = motors.getMotorCommand(FRONT);
        DUMP(i_val);                          //i Motor cmd - assuming stable level hover so only dump one
            break;
            
       case (LOG_REC_BARODATA):
         DUMP(type);
         DUMPA(&currentTime,3);
         b_val = 0; // AKA b_val = auxSwitch.getState();
         DUMP(b_val);
         DUMP(b_val); // AKA DUMP(altitude.tb5);
         //f_val = altitude.getRawData();
         DUMP(altitude.altitude);
         //DUMP(altitude.groundb5);
         //DUMP(altitude.wotAltitude);
         //i_val = (flightAngle.groundZaccel * 1000);
         //DUMP(i_val); //a
         break;
        
       case (LOG_REC_BAROGND):
         DUMP(type);
         DUMPA(&currentTime,3);
         DUMP(altitude.groundTemperature);
         DUMP(altitude.groundPressure);
         DUMP(altitude.groundAltitude);
         break;
*/
       case (LOG_REC_PQR):
         DUMP(type);
         DUMPA(&currentTime,3);
         DUMP(gyro.gyroData[ROLL]);
         DUMP(gyro.gyroData[PITCH]);
         DUMP(gyro.gyroData[YAW]);
         break;

      case (LOG_REC_XYZ):
         DUMP(type);
         DUMPA(&currentTime,3);
         DUMP(accel.accelData[ROLL]);
         DUMP(accel.accelData[PITCH]);
         DUMP(accel.accelData[YAW]);
         break;
         
      case (LOG_REC_HDG):
         DUMP(type);
         DUMPA(&currentTime,3);
         DUMP(accel.accelOneG);
         DUMP(compass.hdgX);
         DUMP(compass.hdgY);
         break;
         
      case (LOG_REC_PT):
         DUMP(type);
         DUMPA(&currentTime,3);
         f_val = flightAngle->getData(ROLL);
         DUMP(f_val);
         f_val = flightAngle->getData(PITCH);
         DUMP(f_val);
         break;

    default:
      break;
    }

    // calculate checksum and write data.  Not for header record.
    if (type != LOG_REC_HEADER) {
      // Calculate & populate checksum
      for (; iptr<bptr; iptr++) cksum += *iptr;
      cksum_b = cksum; //truncate
      *bptr = cksum_b;
      loggerSerialPort -> write(_buffer,((uint16_t) bptr - (uint16_t) &_buffer)+1);
    }
  }


  //
  // Invoked from readSerialCommand to enter interactive command mode
  // 
  void enterInteractiveMode () {  
    telemetryLoop = OFF;  // is this needed now?
    _inCommandMode = ON;
    enterCommandMode(LOG_CMDRETRIES);
Serial.println("entered interactive command mode");
  } // end enterInteractiveMode()
  
  
  //
  // Send escape sequence to OpenLog to get it to drop
  // into command mode. Returns # tries it took to see cmd prompt
  // from OpenLog or 0 if timeout waiting after sending esc sequence
  //
  uint8_t enterCommandMode (uint8_t numTries) {

    uint8_t           i;
    uint32_t          lastCharTime;
    uint32_t          currCharTime;

    for (i=1; i<=numTries; i++) {
      
      loggerSerialPort -> flush();
      loggerSerialPort -> write(LOG_OL_ESCSEQ);
      //loggerSerialPort -> write("\r"); // in case we were already in cmd mode
      
      lastCharTime = millis();
      while (lastCharTime + 50 > (currCharTime = millis())) {
        while (loggerSerialPort -> available()) {
          if (loggerSerialPort -> read() == LOG_OL_PROMPT) {
            Serial.print(LOG_OL_PROMPT);
            return(i);
          }
          else
            lastCharTime = currCharTime;
        }
      }
      
    } // end for()
    
    return(0);
  } // end enterCommandMode()
 

  
  //
  // Invoked from readSerialCommand to enter interactive command mode
  // 
  void exitCommandMode () {
    
    digitalWrite (OPENLOG_PIN, LOW);
    delay(1);
    digitalWrite (OPENLOG_PIN, HIGH);
    delay(2200);
    
    //loggerSerialPort -> write("\rrestart\r");      // tell OpenLog to restart logging
    //delay(200);                                    // give OpenLog time to reset.
    _inCommandMode = OFF;
    dumpRecord(LOG_REC_HEADER);                    // dump decoding header record into log
    delay(10);                                    // give header time to flush before returning
    telemetryLoop = ON;                            

    
  } // end exitCommandMode()



  //
  //  Couples normal Serial0 port to OpenLog Serial port
  //  allowing interaction with OpenLogs command line interface
  //  Note - dont want to do this if we are flying.
  //
  void interactiveCommandMode () {
    uint8_t        c;
    
    if (_inCommandMode == OFF) return;

    // Read user, write to logger
    while (Serial.available()) {
      c=Serial.read();

      // command escape char - exit interactive mode
      if (c == LOG_CMDESC) 
        exitCommandMode();
      else 
        loggerSerialPort -> write(c); 
    }

    // Read logger, write to user   
    while (loggerSerialPort -> available()) {
      c=loggerSerialPort -> read(); 
      Serial.write(c);
    }

  } // end interactiveCommandMode()

};

#endif



