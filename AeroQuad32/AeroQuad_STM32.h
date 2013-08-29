#ifndef _AEROQUAD_STM32_H_
	#define _AEROQUAD_STM32_H_

	#define __STM32__
	#define DEBUG_INIT
	tSerial &Serial = SERIAL_VAR;

	#define ADC_NUMBER_OF_BITS	12
	// ADC czy tu czasem nie chodzi o odczyty z GYRO i ACC 
	// nie zmieniam tego

	// Receiver Declaration
	#if defined (ReceiverPPM) || defined (ReceiverHWPPM)
		#undef ReceiverPPM
		#undef ReceiverHWPPM
		#define RECEIVER_STM32PPM
	#elif defined (ReceiverSBUS)
	#else
		#define RECEIVER_STM32
	#endif

	// Motor declaration
	#define MOTOR_STM32


  #ifdef CameraControl
    #define CameraControl_STM32
  #endif


	#if defined(BOARD_aeroquad32)
		#include "platform_aeroquad32.h"
	#elif defined(BOARD_freeflight)
		#include "platform_freeflight.h"
	#elif defined(BOARD_discovery_f4)
		#include "platform_discoveryf4.h"
	#elif defined(BOARD_maple_mini) // mam nadzieje że wywoła się nasze gówno :)
		#include "platform_maple_mini.h"
	#else
		#error "No motor pinout defined for this STM32 board"
	#endif
#endif

