/*
  ArduinoLog 1.0.0  
  Copyright (c) 2011 Michael C Dillon.  All rights reserved.
  Simple logging utility for the Arduino platform.
 
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


#include "stdarg.h"
     
/**
  * Provides a simple and lightweight logging enviroment for
  * debugging and development purposes. Probably should not
  * be left enabled when flying.
  *
  * Check http://github.com/michaelcdillon/ArduinoLog
  *
  * Supported formatters:
  *     %b - binary value
  *     %d - int / byte
  *     %f - double / float
  *     %s - char string
  *     %x - hex value
  *
  * To enable the Log, place the following define statement in
  * the class you are including everthing in:
  *     #define DEVELOPMENT_LOG
  *
  * To set the log level place the following define statement in
  * the class you are including everything in:
  *     #define LOG_LEVEL <Log_Level>
  *
  * Make sure you have the include statements setup properly after
  * the Log define statements. 
  *
  * Example Usage:
  *
  *     #define  DEVELOPMENT_LOG
  *     #define  LOG_LEVEL INFO_LEVEL
  *     #include Log.h
  *     
  *     Log::fine ("Fine statement");
  *     Log::debug ("Debug statement");
  *     Log::info ("Info statement ");
  *     Log::warn ("Warn statement");
  *     Log::error ("Error Statement");
  *
  *     Log::info ("Int: %d", 5);
  *     Log::info ("Double: %f", 5.5);
  *     Log::info ("Chars: %s", "test string");
  *     Log::info ("Binary of 5: %b", 5);
  *     Log::info ("Hex of 47: %x", 47);
  *        
  *     Log::info ("Unrecognized formatter: %p", 1);
  *
  */

#define LOG_ALL       0
#define FINE_LEVEL    0
#define DEBUG_LEVEL   1
#define INFO_LEVEL    2
#define WARN_LEVEL    3
#define ERROR_LEVEL   4
#define LOG_OFF       5

class Log {
private:
    static const char fine_prefix[];
    static const char debug_prefix[];
    static const char info_prefix[];
    static const char warn_prefix[];
    static const char error_prefix[]; 
    
    /**
      * Prints the prefix of the message and the actual message
      * to the standard Serial port.
      */ 
    static void print_msg (const char prefix[], char msg[], va_list ap) {
    #ifdef DEVELOPMENT_LOG
        float   temp_float;
        int     temp_int;
        char*   temp_chars;
        int     i = 0;
        char    cur;

        Serial.print (prefix);

        while ((cur =  msg[i]) != '\0') {
            if (cur != '%')  {
                Serial.print (cur);
                i++;
            }
            else { 
                i++;
                cur = msg[i];
                
                switch (cur) {
                    case 'b':
                            temp_int = va_arg (ap, int);
                            Serial.print (temp_int, BIN);
                            break;
                    case 'd':
                            temp_int = va_arg (ap, int);
                            Serial.print (temp_int);
                            break;
                    case 'f':
                            temp_float = va_arg (ap, double);
                            Serial.print (temp_float);
                            break;
                    case 's':
                            temp_chars = va_arg (ap, char*);
                            Serial.print (temp_chars);
                            break;
                    case 'x':
                            temp_int = va_arg (ap, int);
                            Serial.print (temp_int, HEX);
                            break;
                    default:
                            Serial.print ("<Log Error: Unrecognized Formatter: %");
                            Serial.print (cur);
                            Serial.print (" >");
                        break;                 
                }
                i++;
            }
        }

        Serial.print ('\n');
    #endif
    }

public:
    
    /**
      * Prints a prefixed fine statment message
      */
    static void fine (char msg[], ...) {
    #ifdef DEVELOPMENT_LOG
        if (LOG_LEVEL <=  FINE_LEVEL) {
            va_list ap;
            va_start (ap, msg);
            print_msg (fine_prefix, msg, ap);
            va_end (ap);
        }    
    #endif
    }

    /**
      * Prints a prefixed debug statment message
      */
    static void debug (char msg[], ...) {
    #ifdef DEVELOPMENT_LOG
        if (LOG_LEVEL <=  DEBUG_LEVEL) {
            va_list ap;
            va_start (ap, msg);
            print_msg (debug_prefix, msg, ap);
            va_end (ap);
        }    
    #endif
    }

    /**
      * Prints a prefixed info statment message
      */
    static void info (char msg[], ...) {
    #ifdef DEVELOPMENT_LOG
        if (LOG_LEVEL <=  INFO_LEVEL) {
            va_list ap;
            va_start (ap, msg);
            print_msg (info_prefix, msg, ap);
            va_end (ap);
        }
    #endif
    }

    /**
      * Prints a prefixed warn statment message
      */
    static void warn (char msg[], ...) {
    #ifdef DEVELOPMENT_LOG
        if (LOG_LEVEL <= WARN_LEVEL) {
            va_list ap;
            va_start (ap, msg);
            print_msg (warn_prefix, msg, ap);
            va_end (ap);
        }
    #endif
    }
    
    /**
      * Prints a prefixed error statment message
      */
    static void error (char msg[], ...) {
    #ifdef DEVELOPMENT_LOG
        if (LOG_LEVEL <= ERROR_LEVEL) {
            va_list ap;
            va_start (ap, msg);
            print_msg (error_prefix, msg, ap);
            va_end (ap);
        }
    #endif
    }
};

#ifdef DEVELOPMENT_LOG
    const char Log::fine_prefix[]   = "[FINE] ";
    const char Log::debug_prefix[]  = "[DEBUG] ";
    const char Log::info_prefix[]   = "[INFO] ";
    const char Log::warn_prefix[]   = "[WARN] ";
    const char Log::error_prefix[]  = "[ERROR] ";
#endif
