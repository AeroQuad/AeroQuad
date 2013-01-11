#ifndef _AEROQUAD_MOTORS_STM32_H_
#define _AEROQUAD_MOTORS_STM32_H_

#if defined(AeroQuadSTM32)

#include "Motors.h"

////////////////////////////////////////////////////////
// definition section

#define PWM_FREQUENCY   400      // Hz
#define PWM_PERIODE     (1000000/PWM_FREQUENCY)

#ifdef MOTORS_STM32_TRI
  #define PWM_SERVO_FREQUENCY 50 // Hz 
  #define PWM_SERVO_PERIODE   (1000000/PWM_SERVO_FREQUENCY)
  #define STM32_MOTOR_MAP stm32_motor_mapping_tri
#else
  #define STM32_MOTOR_MAP stm32_motor_mapping
#endif

////////////////////////////////////////////////////////
// code section

static int _stm32_motor_number;

// global section

void initializeMotors(NB_Motors numbers) {
 
  int motor;

  _stm32_motor_number = sizeof(STM32_MOTOR_MAP)/sizeof(STM32_MOTOR_MAP[0]);
  if(numbers < _stm32_motor_number) {
    _stm32_motor_number = numbers;
  }
  
  for(motor=0; motor < _stm32_motor_number; motor++) {

    int prescaler = rcc_dev_timer_clk_speed(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device->clk_id)/1000000 - 1;

    timer_set_prescaler(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, prescaler);

#ifdef MOTORS_STM32_TRI
    // on Tri mode motor 0 is a servo and thus has slower update rate
    if (motor == 0) {
      timer_set_reload(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PWM_SERVO_PERIODE);
    }
    else {
      timer_set_reload(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PWM_PERIODE);
    }
#else
    timer_set_reload(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PWM_PERIODE);
#endif
    
    pinMode(STM32_MOTOR_MAP[motor], PWM);
  }
  
  // sync timer
  for(motor=0; motor < _stm32_motor_number; motor++) {
    timer_generate_update(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device);
  }
  
  commandAllMotors(1000);
  //Serial.println("motor init done\r\n");
}

void writeMotors(void) { // update motor commands on timers

  for(int motor=0; motor < _stm32_motor_number; motor++) {
    timer_set_compare(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PIN_MAP[STM32_MOTOR_MAP[motor]].timer_channel,  motorCommand[motor]);
  }
}

void commandAllMotors(int _motorCommand) {   // Send same command to all motors

  for(int motor=0; motor < _stm32_motor_number; motor++) {
    timer_set_compare(PIN_MAP[STM32_MOTOR_MAP[motor]].timer_device, PIN_MAP[STM32_MOTOR_MAP[motor]].timer_channel, _motorCommand);
  }
}

#endif
#endif
