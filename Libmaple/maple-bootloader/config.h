/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 LeafLabs LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

/**
 *  @file config.h
 *
 *  @brief bootloader settings and macro defines
 *
 *
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "common.h"


/* On the Native, LED is PC15, now take PE5 */
#ifdef BOARD_maple_native
#define FLASH_PAGE_SIZE  0x800 /* 2KB pages for high density devices */
// LED is PC15
#define LED_BANK         GPIOC
#define LED              15
#define LED_BANK_CR      GPIO_CRH(LED_BANK)
#define LED_CR_MASK      0x0FFFFFFF
#define LED_CR_OUTPUT    0x10000000
#define RCC_APB2ENR_LED  0x00000010 /* enable PC */

/* On the Native, BUT is PG15, now PC0 */
#define BUTTON_BANK      GPIOG
#define BUTTON           15
#define BUT_BANK_CR      GPIO_CRH(BUTTON_BANK)
#define BUT_CR_MASK      0x0FFFFFFF
#define BUT_CR_INPUTMODE    0x40000000	// input floating
#define RCC_APB2ENR_BUT  0x00000100 /* enable PG */
#define BUTTON_SETUP     resetPin
#endif


#ifdef BOARD_aeroquad32
	#define FLASH_PAGE_SIZE  0x800 /* 2KB pages for high density devices */
	#define USER_CODE_FLASH  ((u32)0x08010000)   /* ala42 */

	// LED is PE5
	#define LED_BANK         GPIOE
	#define LED              5
	#define LED_BANK_CR      GPIO_CRL(LED_BANK)
	#define LED_CR_MASK      0xFF0FFFFF
	#define LED_CR_OUTPUT    0x00100000
	#define RCC_APB2ENR_LED  0x00000040 /* enable PE */
	#define RCC_AHB1ENR_LED  0x00000010 /* F2 enable PE */

	/* BUT is PD15 */
	#define BUTTON_BANK      GPIOD
	#define BUTTON           15				   // PD15
	#define BUT_BANK_CR      GPIO_CRH(BUTTON_BANK)
	#define BUT_CR_MASK      0x0FFFFFFF
	//#define BUT_CR_INPUTMODE    0x00000004 // input floating
	#define BUT_CR_INPUTMODE    0x80000000 // input pull up/down, output pin has to be set for pull up, cleared for pull down
	#define RCC_APB2ENR_BUT  0x00000020 /* enable PD */
	#define RCC_AHB1ENR_BUT  0x00000008 /* F2 enable PE */
	#define BUTTON_SETUP     resetPin

	#if 0
		/* BUT is PC0 */
		#define BUTTON_BANK      GPIOC
		#define BUTTON           0					// PC0
		#define BUT_BANK_CR      GPIO_CRL(BUTTON_BANK)
		#define BUT_CR_MASK      0xFFFFFFF0
		//#define BUT_CR_INPUTMODE    0x00000004 // input floating
		#define BUT_CR_INPUTMODE    0x00000008 // input pullup, output pin has to be set
		#define RCC_APB2ENR_BUT  0x00000010 /* enable PC */
		#define RCC_AHB1ENR_BUT  0x00000004 /* F2 enable PC */
		#define BUTTON_SETUP     setPin
	#endif
#endif

#ifdef BOARD_DiscoveryF4
	#define FLASH_PAGE_SIZE  0x800 /* 2KB pages for high density devices */
	#define USER_CODE_FLASH  ((u32)0x08010000)   /* ala42 */

	// LED is PD13
	#define LED_BANK         GPIOD
	#define LED              13
	#define LED_BANK_CR      GPIO_CRH(LED_BANK)
	#define LED_CR_MASK      0xFF0FFFFF
	#define LED_CR_OUTPUT    0x00100000
	//#define RCC_APB2ENR_LED  0x00000020 /* enable PD */
	#define RCC_AHB1ENR_LED  0x00000008 /* F2 enable PD */

	/* BUT is PA0 */
	#define BUTTON_BANK      GPIOA
	#define BUTTON           0				   // PA0
	//#define BUT_BANK_CR      GPIO_CRL(BUTTON_BANK)
	//#define BUT_CR_MASK      0xFFFFFFF0
	//#define BUT_CR_INPUTMODE    0x00000004 // input floating
	//#define BUT_CR_INPUTMODE    0x00000008 // input pull up/down, output pin has to be set for pull up, cleared for pull down
	#define RCC_AHB1ENR_BUT  0x00000001 /* F2 enable PA */
	//#define BUTTON_SETUP     resetPin
#endif




#ifdef BOARD_aeroquad32mini
	#define FLASH_PAGE_SIZE  0x400 /* 1KB pages for medium density devices */
	#define USER_CODE_FLASH  ((u32)0x08005000)  /* ala42 */
	// LED is PA3
	#define LED_BANK         GPIOA
	#define LED              3
	#define LED_BANK_CR      GPIO_CRL(LED_BANK)
	#define LED_CR_MASK      0xFFFF0FFF
	#define LED_CR_OUTPUT    0x00001000
	#define RCC_APB2ENR_LED  0x00000004 /* enable PA */

	/* BUT is PA4 */
	#define BUTTON_BANK      GPIOA
	#define BUTTON           4					// PA4
	#define BUT_BANK_CR      GPIO_CRL(BUTTON_BANK)
	#define BUT_CR_MASK      0xFFF0FFFF
	//#define BUT_CR_INPUTMODE    0x00000004 // input floating
	#define BUT_CR_INPUTMODE 0x00080000 // input pull up/down, output pin has to be set for pull up, cleared for pull down
	#define RCC_APB2ENR_BUT  0x00000004 /* enable PA */
	#define BUTTON_SETUP     resetPin
#endif



/* Speed controls for strobing the LED pin */
#ifdef STM32F2
  #define BLINK_FAST       (2*0x50000)
  #define BLINK_SLOW       (2*0x100000)
#else
  #define BLINK_FAST       0x50000
  #define BLINK_SLOW       0x100000
#endif

#ifdef FASTSTART
#define BOOTLOADER_WAIT  0
#define STARTUP_BLINKS   2
#else
#define BOOTLOADER_WAIT  3 // ala42: was 6
#define STARTUP_BLINKS   5
#endif

#define USER_CODE_RAM    ((u32)0x20000C00)
#define BUILT_IN_BOOT_LOADER ((u32)0x1FFF0000)
#define START_BOOT_LOADER_MAGIC_ADDR ((u32*)(USER_CODE_RAM-4))
#define START_BOOT_LOADER_MAGIC (0x4AFC6BB2)

#define VEND_ID0         0xAF
#define VEND_ID1         0x1E
#define PROD_ID0         0x03
#define PROD_ID1         0x00

#endif
