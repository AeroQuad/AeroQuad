#ifndef _AEROQUAD_MOTORS_STM32_H_
#define _AEROQUAD_MOTORS_STM32_H_

#if defined(AeroQuadSTM32)

#include "Motors.h"

////////////////////////////////////////////////////////
// definition section

//#if defined (USE_400HZ_ESC)
  #define PWM_FREQUENCY 400   // in Hz
//#else
  //#define PWM_FREQUENCY 300   // in Hz
//#endif
#define PWM_PERIODE     (1000000/PWM_FREQUENCY)


static byte __attribute__((unused)) stm32_motor_mapping[] = {
  Port2Pin('C',  9),
  Port2Pin('C',  8),
  Port2Pin('C',  7),
  Port2Pin('C',  6),
  Port2Pin('A', 15),
  Port2Pin('B',  3),
  Port2Pin('B',  4),
  Port2Pin('B',  5)
};

static byte __attribute__((unused)) stm32_motor_mapping_tri[] = {
  Port2Pin('A', 15), // note this must be on separate timer device !!
  Port2Pin('C',  8),
  Port2Pin('C',  7),
  Port2Pin('C',  6),
};



#define PWM_SERVO_FREQUENCY 50 // Hz 

#define PWM_SERVO_PERIODE   (1000000/PWM_SERVO_FREQUENCY)
#define STM32_MOTOR_MAP_TRI stm32_motor_mapping_tri
#define STM32_MOTOR_MAP stm32_motor_mapping

////////////////////////////////////////////////////////
// code section

static int _stm32_motor_number;

// global section

void initializeMotors(byte numbers) {
 
  int motor;
  
  if (flightConfigType == TRI) {
    
	_stm32_motor_number = sizeof(STM32_MOTOR_MAP_TRI)/sizeof(STM32_MOTOR_MAP_TRI[0]);
    if(numbers < _stm32_motor_number) {
      _stm32_motor_number = numbers;
    }
  
    for(motor=0; motor < _stm32_motor_number; motor++) {

      int prescaler = rcc_dev_timer_clk_speed(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device->clk_id)/1000000 - 1;

      timer_set_prescaler(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device, prescaler);

      // on Tri mode motor 0 is a servo and thus has slower update rate
      if (motor == 0) {
        timer_set_reload(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device, PWM_SERVO_PERIODE);
      }
      else {
        timer_set_reload(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device, PWM_PERIODE);
      }
    
      pinMode(STM32_MOTOR_MAP_TRI[motor], PWM);
    }
  
    // sync timer
    for(motor=0; motor < _stm32_motor_number; motor++) {
      timer_generate_update(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device);
    }
  }
  else {
  
    _stm32_motor_number = sizeof(STM32_MOTOR_MAP)/sizeof(STM32_MOTOR_MAP[0]);
    if(numbers < _stm32_motor_number) {
      _stm32_motor_number = numbers;
    }
  
    for(motor=0; motor < _stm32_motor_number; motor++) {

      int prescaler = rcc_dev_timer_clk_speed(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device->clk_id)/1000000 - 1;

      timer_set_prescaler(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, prescaler);

     
      timer_set_reload(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PWM_PERIODE);
    
      pinMode(STM32_MOTOR_MAP[motor], PWM);
    }
  
    // sync timer
    for(motor=0; motor < _stm32_motor_number; motor++) {
      timer_generate_update(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device);
    }
  }

  commandAllMotors(1000);
  //Serial.println("motor init done\r\n");
}

void writeMotors(void) { // update motor commands on timers

  if (flightConfigType == TRI) {
    for(int motor=0; motor < _stm32_motor_number; motor++) {
      timer_set_compare(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device, PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_channel,  motorCommand[motor]);
    }
  }
  else {
    for(int motor=0; motor < _stm32_motor_number; motor++) {
      timer_set_compare(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PIN_MAP[STM32_MOTOR_MAP[motor]].timer_channel,  motorCommand[motor]);
    }
  }
  
}

void commandAllMotors(int _motorCommand) {   // Send same command to all motors

  if (flightConfigType == TRI) {
    for(int motor=0; motor < _stm32_motor_number; motor++) {
      timer_set_compare(PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_device, PIN_MAP[STM32_MOTOR_MAP_TRI[motor]].timer_channel, _motorCommand);
    }
  }
  else {
    for(int motor=0; motor < _stm32_motor_number; motor++) {
      timer_set_compare(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PIN_MAP[STM32_MOTOR_MAP[motor]].timer_channel, _motorCommand);
    }
  }
}

#endif
#endif
