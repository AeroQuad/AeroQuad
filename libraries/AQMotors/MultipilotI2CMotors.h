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

#ifndef _AQ_MULTIPILOT_I2C_MOTORS_H_
#define _AQ_MULTIPILOT_I2C_MOTORS_H_

#include "Motors.h"


class MultipilotI2CMotors : public Motors 
{
public:

/* Mixertable VAR */
  float MotorGas[LASTMOTOR];
  float MotorPitch[LASTMOTOR];
  float MotorRoll[LASTMOTOR];
  float MotorYaw[LASTMOTOR];
  float motorAxisCommandPitch[LASTMOTOR];
  float motorAxisCommandRoll[LASTMOTOR];
  float motorAxisCommandYaw[LASTMOTOR];

  unsigned char MotorI2C[LASTMOTOR];
  MultipilotI2CMotors() : Motors() {}

  void initialize() 
  {
    char Motor[LASTMOTOR];
    // Scale motor commands to analogWrite
    // m = (250-126)/(2000-1000) = 0.124
    // b = y1 - (m * x1) = 126 - (0.124 * 1000) = 2
    //float _mMotorCommand = 0.124;
    //float _bMotorCommand = 2 ;

    _mMotorCommand = 0.255;
    _bMotorCommand = -255;
    timer_debug=0;

    //motorCommand[8] = {1000,1000,1000,1000,1000,1000,1000,1000};

    //int subtrim[4] = {1500,1500,1500,1500};    //SUBTRIM li esprimo in millisecondi come i servi per standardizzazione.
    //motorAxisCommand[3] = {0,0,0};

    // If AREF = 3.3V, then A/D is 931 at 3V and 465 = 1.5V
    // Scale gyro output (-465 to +465) to motor commands (1000 to 2000)
    // use y = mx + b
    //mMotorRate = 1.0753; // m = (y2 - y1) / (x2 - x1) = (2000 - 1000) / (465 - (-465))
    //bMotorRate = 1500;   // b = y1 - m * x1
    init_mixer_table(); // Init MixerTable
    Wire.begin(0x29);
 }

// C'e' im
#ifdef HEXARADIAL
void init_mixer_table()
{
  // Example for Hexa configuration
  MotorGas[0] = 100;
  MotorPitch[0] = -100;
  MotorRoll[0] =  0;
  MotorYaw[0] =  100;
  
  MotorGas[1] = 100;
  MotorPitch[1] = -50;
  MotorRoll[1] =  -100;
  MotorYaw[1] =  -100;
  
  MotorGas[2] = 100;
  MotorPitch[2] = +50  ;
  MotorRoll[2] =  -100;
  MotorYaw[2] =  100;
  
  MotorGas[3] = 100;
  MotorPitch[3] =  +100;
  MotorRoll[3] =  0;
  MotorYaw[3] =  -100;
  
  MotorGas[4] = 100;
  MotorPitch[4] = +50;
  MotorRoll[4] =  100;
  MotorYaw[4] =  100;
  
  MotorGas[5] = 100;
  MotorPitch[5] =  -50;
  MotorRoll[5] =  100;
  MotorYaw[5] =  -100;
}
#endif

// Example of Hexa Coaxial.
//Configurazione Motori Hexa Coaxial

//Dietro 			GAS		NICK		        ROLL		        YAW
//Sopra 1         		64		-64			0			-64
//Sotto 2			76		-64			0			64

//Guardando l'Hexafox da dietro braccio  Sinistro
//Sopra 3			64		32			64			64
//Sotto 4			76		32			64			-64

//Guardando l'Hexafox da dietro braccio  Destro.

//Sopra 5			64		32			-64			64
//Sotto 6			76		32			-64			-64


#ifdef HEXACOAXIAL
void init_mixer_table()
{
  // Example for Hexa configuration
  MotorGas[0] = 95;
  MotorPitch[0] = 100;
  MotorRoll[0] =  0;
  MotorYaw[0] =  100;
  
  MotorGas[1] = 100;
  MotorPitch[1] = 100;
  MotorRoll[1] =  0;
  MotorYaw[1] =  -100;
  
  MotorGas[2] = 95;
  MotorPitch[2] = -50  ;
  MotorRoll[2] =  100;
  MotorYaw[2] =  -100;
  
  MotorGas[3] = 100;
  MotorPitch[3] =  -50;
  MotorRoll[3] =  100;
  MotorYaw[3] =  100;
  
  MotorGas[4] = 95;
  MotorPitch[4] = -50;
  MotorRoll[4] =  -100;
  MotorYaw[4] =  -100;
  
  MotorGas[5] = 100;
  MotorPitch[5] =  -50;
  MotorRoll[5] =  -100;
  MotorYaw[5] =  100;
}
#endif


void motor_axis_correction()
{
  byte i;
  for (i=0;i<LASTMOTOR;i++)
  {
    motorAxisCommandPitch[i] = (motorAxisCommand[PITCH] / 100.0) * MotorPitch[i];
    motorAxisCommandRoll[i] = (motorAxisCommand[ROLL] / 100.0) * MotorRoll[i];
    motorAxisCommandYaw[i] = (motorAxisCommand[YAW] / 100.0) * MotorYaw[i];
  }
}

//After that we can mix them together:
void motor_matrix_command()
{
  int i;
  float valuemotor;
  for (i=0;i<LASTMOTOR;i++)
  {
     valuemotor = ((Throttle* MotorGas[i])/100) + motorAxisCommandPitch[i] + motorAxisCommandYaw[i] + motorAxisCommandRoll[i];
     //valuemotor = Throttle + motorAxisCommandPitch[i] + motorAxisCommandYaw[i] + motorAxisCommandRoll[i]; // OLD VERSION WITHOUT GAS CONTROL ON Mixertable
     valuemotor = constrain(valuemotor, minAcro, MAXCOMMAND);
     motorCommand[i]=valuemotor;
  }
}


void matrix_debug()
{
#ifdef PRINT_MIXERTABLE
     Serial.println();
     Serial.println("--------------------------");
     Serial.println("        Motori Mixertable " );
     Serial.println("--------------------------");
     Serial.println();

/*
     Serial.print("AIL:");
     Serial.print(ch1);
     Serial.print(" ELE:");
     Serial.print(ch2);
*/
     Serial.print(" THR:");
     Serial.print(Throttle);
/*
     Serial.print(" YAW:");
     Serial.print(ch4);
     Serial.print(" AUX:");
     Serial.print(ch_aux);
     Serial.print(" AUX2:");
     Serial.print(ch_aux2);
  */
     Serial.println();
     Serial.print("CONTROL_ROLL:");
     Serial.print(motorAxisCommand[ROLL]);
     Serial.print(" CONTROL_PITCH:");
     Serial.print(motorAxisCommand[PITCH]);
     Serial.print(" CONTROL_YAW:");
     Serial.print(motorAxisCommand[YAW]);
//     Serial.print(" SONAR_VALUE:");
//     Serial.print(sonar_value);
//     Serial.print(" TARGET_SONAR_VALUE:");
//     Serial.print(target_sonar_altitude);
//     Serial.print(" ERR_SONAR_VALUE:");
//     Serial.print(err_altitude);
//     Serial.println();
//     Serial.print("latitude:");
//     Serial.print(GPS_np.Lattitude);
//     Serial.print(" longitude:");
//     Serial.print(GPS_np.Longitude);
//     Serial.print(" command gps roll:");
//     Serial.print(command_gps_roll);
//     Serial.print(" command gps pitch:");
//     Serial.print(command_gps_pitch);
//     Serial.print(" Lon_diff:");
//     Serial.print(Lon_diff);
//     Serial.print(" Lon_diff");
//     Serial.print(command_gps_pitch);

//     Serial.println();

//     Serial.print("AP MODE:");Serial.print((int)AP_mode);

#ifdef ARDUCOPTER
     Serial.print("AIL:");
     Serial.print(ch1);
     Serial.print(" ELE:");
     Serial.print(ch2);
     Serial.print(" THR:");
     Serial.print(ch3);
     Serial.print(" YAW:");
     Serial.print(ch4);
     Serial.print(" AUX:");
     Serial.print(ch_aux);
     Serial.print(" AUX2:");
     Serial.print(ch_aux2);
     Serial.println();
     Serial.print("CONTROL_ROLL:");
     Serial.print(control_roll);
     Serial.print(" CONTROL_PITCH:");
     Serial.print(control_pitch);
     Serial.print(" CONTROL_YAW:");
     Serial.print(control_yaw);
     Serial.print(" SONAR_VALUE:");
     Serial.print(sonar_value);
     Serial.print(" TARGET_SONAR_VALUE:");
     Serial.print(target_sonar_altitude);
     Serial.print(" ERR_SONAR_VALUE:");
     Serial.print(err_altitude);
     Serial.println();
     Serial.print("latitude:");
     Serial.print(GPS_np.Lattitude);
     Serial.print(" longitude:");
     Serial.print(GPS_np.Longitude);
     Serial.print(" command gps roll:");
     Serial.print(command_gps_roll);
     Serial.print(" command gps pitch:");
     Serial.print(command_gps_pitch);
     Serial.print(" Lon_diff:");
     Serial.print(Lon_diff);
     Serial.print(" Lon_diff");
     Serial.print(command_gps_pitch);

     Serial.println();

     Serial.print("AP MODE:");Serial.print((int)AP_mode);
#endif

#ifdef HEXARADIAL
     Serial.println();
     Serial.print((unsigned int)MotorI2C[5]);
     comma();
     Serial.print((unsigned int)MotorI2C[0]);
     comma();
     Serial.print((unsigned int)MotorI2C[1]);
     comma();
     Serial.println();
     Serial.print((unsigned int)MotorI2C[4]);
     comma();
     Serial.print((unsigned int)MotorI2C[3]);
     comma();
     Serial.println((unsigned int)MotorI2C[2]);
     Serial.println("---------------");
     Serial.println();
#endif

  // Example of Hexa Coaxial.
  //Configurazione Motori Hexa Coaxial
  
  //Dietro 			GAS		NICK		        ROLL		        YAW
  //Sopra 1         		64		-64			0			-64
  //Sotto 2			76		-64			0			64
  
  //Guardando l'Hexafox da dietro braccio  Sinistro
  //Sopra 3			64		32			64			64
  //Sotto 4			76		32			64			-64
  
  //Guardando l'Hexafox da dietro braccio  Destro.
  
  //Sopra 5			64		32			-64			64
  //Sotto 6			76		32			-64			-64
  
  
  #ifdef HEXACOAXIAL
       Serial.println();
       Serial.print((unsigned int)MotorI2C[2]);
       comma();
       Serial.print((unsigned int)MotorI2C[4]);
       Serial.println();
       //comma();
       Serial.print((unsigned int)MotorI2C[3]);
       comma();
       Serial.print((unsigned int)MotorI2C[5]);
       Serial.println();
       Serial.print ("   ");
       //comma();
       Serial.print((unsigned int)MotorI2C[0]);
       Serial.println();
       Serial.print ("   ");
       //comma();
       Serial.println((unsigned int)MotorI2C[1]);
       Serial.println("---------------");
       Serial.println();
  #endif
  
  #endif
  }
  
  void WireMotorWrite()
  {
    int i = 0;
    int nmotor=0;
    int index=0;
    int tout=0;
    char buff_i2c[10];
  
    Wire.endTransmission(); //end transmission
    for(nmotor=0;nmotor<6;nmotor++)
    {
      index=0x29+nmotor;
      Wire.beginTransmission(index);
      Wire.send(MotorI2C[nmotor]);
      Wire.endTransmission(); //end transmission
      Wire.requestFrom(index, 1);    // request 6 bytes from device
      i=0;
      while(1)
      //while((Wire.available())&&(i<6))
      {
        buff_i2c[i] = Wire.receive();  // receive one byte
        i++;
        if (i>6)
        {
          break;
        }
        //Serial.print(i);
        if (Wire.available()==0)
        {
          break;
        }
      }
    }
  }
  void write () 
  {
    // Matrix transformation.
    motor_axis_correction();
    // Matrix Command.
    motor_matrix_command();
   // Matrix Assignment.
    MotorI2C[MOTORID1]=(char)((motorCommand[0] * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID2]=(char)((motorCommand[1] * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID3]=(char)((motorCommand[2] * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID4]=(char)((motorCommand[3] * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID5]=(char)((motorCommand[4] * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID6]=(char)((motorCommand[5] * _mMotorCommand) + _bMotorCommand);
  
    if((millis()-timer_debug)>1000)   // 100ms => 10 Hz loop rate
    {
      timer_debug=millis();
      #ifdef TELEMETRY_DEBUG
      matrix_debug();
      #endif
    }
    WireMotorWrite();
  }
  
  void commandAllMotors(int _motorCommand) 
  {   // Sends commands to all motors
    // Matrix transformation.
    motor_axis_correction();
    // Matrix Command.
    motor_matrix_command();
   // Matrix Assignment.
    MotorI2C[MOTORID1]=(char)((_motorCommand * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID2]=(char)((_motorCommand * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID3]=(char)((_motorCommand * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID4]=(char)((_motorCommand * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID5]=(char)((_motorCommand * _mMotorCommand) + _bMotorCommand);
    MotorI2C[MOTORID6]=(char)((_motorCommand * _mMotorCommand) + _bMotorCommand);
    matrix_debug();
    WireMotorWrite();
  }
};

#endif