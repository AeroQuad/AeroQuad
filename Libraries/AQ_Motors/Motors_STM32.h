#ifndef _AEROQUAD_MOTORS_STM32_H_
#define _AEROQUAD_MOTORS_STM32_H_

#if defined(AeroQuadSTM32)

#include "Motors.h"

////////////////////////////////////////////////////////
// definition section

#define PWM_FREQUENCY 400 // Hz

#if defined(BOARD_aeroquad32)
  static byte stm32_motor_mapping[] = {
    Port2Pin('C',  9),
    Port2Pin('C',  8),
    Port2Pin('C',  7),
    Port2Pin('C',  6),
    Port2Pin('A', 15),
    Port2Pin('B',  3),
    Port2Pin('B',  4),
    Port2Pin('B',  5)
  };
#elif defined(BOARD_aeroquad32mini)
  static byte stm32_motor_mapping[] = {3, 9, 10, 11, 12, 13};
#elif defined(BOARD_freeflight)
  static byte stm32_motor_mapping[] = {
    Port2Pin('B',  6),
    Port2Pin('B',  7),
    Port2Pin('B',  8),
    Port2Pin('B',  9),
    Port2Pin('A',  8),
    Port2Pin('A', 11)
  };
#elif defined(BOARD_discovery_f4)
  static byte stm32_motor_mapping[] = {
    Port2Pin('C',  9),
    Port2Pin('C',  8),
    Port2Pin('C',  7),
    Port2Pin('C',  6),
	// pin mapping for motor 5-8 not specified, yet
    Port2Pin('A', 15),
    Port2Pin('B',  3),
    Port2Pin('B',  4),
    Port2Pin('B',  5)
  };
#else
  #error "No motor pinout defined for this STM32 board"
#endif


////////////////////////////////////////////////////////
// code section

#define PWM_PERIODE (1000000/PWM_FREQUENCY)

  // private section
  static int _stm32_motor_number;

  struct _sMotorInfo {
	int			pin;
    timer_dev 	*timer_device;	/**< Pin's timer device, if any. */
    uint8 		timer_channel;  /**< Timer channel, or 0 if none. */
  } MotorInfo[8];

  void _initMotorInfo(int motor, int pin) {
	  MotorInfo[motor].pin           = pin;
	  MotorInfo[motor].timer_device  = PIN_MAP[pin].timer_device;
	  MotorInfo[motor].timer_channel = PIN_MAP[pin].timer_channel;
  }

  // global section

  void initializeMotors(NB_Motors numbers) {
	_stm32_motor_number = sizeof(stm32_motor_mapping)/sizeof(stm32_motor_mapping[0]);
	if(numbers < _stm32_motor_number) {
		_stm32_motor_number = numbers;
	}

	int motor;
    for(motor=0; motor < _stm32_motor_number; motor++) {
    	_initMotorInfo(motor, stm32_motor_mapping[motor]);

    	int prescaler = rcc_dev_timer_clk_speed(MotorInfo[motor].timer_device->clk_id)/1000000 - 1;
    	timer_set_prescaler(MotorInfo[motor].timer_device, prescaler);
    	timer_set_reload(MotorInfo[motor].timer_device, PWM_PERIODE);

    	pinMode(MotorInfo[motor].pin, PWM);
    }

    // sync timer
    for(motor=0; motor < _stm32_motor_number; motor++) {
    	timer_generate_update(MotorInfo[motor].timer_device);
    }

    commandAllMotors(1000);
	//Serial.println("motor init done\r\n");
  }

  void writeMotors(void) {
	for(int motor=0; motor < _stm32_motor_number; motor++) {
	   	timer_set_compare(MotorInfo[motor].timer_device, MotorInfo[motor].timer_channel,  motorCommand[motor]);
	}
  }

  void commandAllMotors(int _motorCommand) {   // Sends commands to all motors
	for(int motor=0; motor < _stm32_motor_number; motor++) {
	   	timer_set_compare(MotorInfo[motor].timer_device, MotorInfo[motor].timer_channel, _motorCommand);
	}
  }

/*
 * test commands for motor testing
  5123.45;1010;1020;1030;1040;1050;1060;1070;1080
  5123.45;1011;1021;1031;1041;1051;1061;1071;1081
  5123.45;1110;1120;1130;1140;1150;1160
  5123.45;1000;1000;1000;1000;1000;1000
*/
#endif
#endif
