#ifndef WProgram_h
	#define WProgram_h
	//#define USE_USB_SERIAL


	extern void setup();
	extern void loop();

        #include <stdint.h>
	#include <stdlib.h>
	#include <string.h>
	#include <math.h>

	#include <wirish/wirish.h>
	#define cli()     nvic_globalirq_disable()
	#define sei()     nvic_globalirq_enable()

	#ifdef __cplusplus
	  #ifdef BOARD_freeflight
	    #undef USE_USB_SERIAL
	  #endif
		#ifdef USE_USB_SERIAL
			#define SERIAL_VAR SerialUSB
			typedef USBSerial tSerial;
		#else
			#ifdef BOARD_discovery_f4
				#define SERIAL_VAR Serial3
			#else
				#define SERIAL_VAR Serial1
			#endif
			typedef HardwareSerial tSerial;
		#endif

		extern tSerial &Serial;
		extern USBSerial SerialUSB;

		uint16_t makeWord(uint16_t w);
		uint16_t makeWord(byte h, byte l);

		#define word(...) makeWord(__VA_ARGS__)

		unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout = 1000000L);

		void tone(uint8_t _pin, unsigned int frequency, unsigned long duration = 0);
		void noTone(uint8_t _pin);

		// WMath prototypes
		long random(long);
		long random(long, long);
		void randomSeed(unsigned int);
		long map(long, long, long, long, long);
	#endif
#endif
