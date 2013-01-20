/******************************************************************************
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
 *****************************************************************************/

/**
 *  @file main.c
 *
 *  @brief main loop and calling any hardware init stuff. timing hacks
 *  for EEPROM writes not to block usb interrupts. logic to handle 2
 *  second timeout then jump to user code.
 *
 */

#include "common.h"
#include "dfu.h"


int main() {
  systemReset(); // peripherals but not PC
  setupCLK();
  setupLED();
  setupBUTTON();

#ifdef STM32F2
	setupDisconnectPin();
	disconnectUSB();
#else
  setupUSB();
  setupFLASH();
#endif


  strobePin(LED_BANK,LED,STARTUP_BLINKS,BLINK_FAST);

#ifdef STM32F2
  // enter built in boot load if magic number found in RAM
	if(*START_BOOT_LOADER_MAGIC_ADDR == START_BOOT_LOADER_MAGIC) {
 	  connectUSB();
		*START_BOOT_LOADER_MAGIC_ADDR = 0;
    jumpToUser(BUILT_IN_BOOT_LOADER);
	}
#endif



#ifdef FASTSTART
  // for an unknown reason the loader process hangs if the button port is not read
	bool pinState = readPin(BUTTON_BANK, BUTTON);
#else

	bool buttonStatusHigh = TRUE;
	bool buttonStatusLow  = FALSE;
	for(int i=0; i<10; i++) {
		bool pinState = readPin(BUTTON_BANK, BUTTON);
		buttonStatusHigh = buttonStatusHigh && pinState;
		buttonStatusLow  = buttonStatusLow || pinState;
	}

  /* wait for host to upload program or halt bootloader */
  bool no_user_jump = (!checkUserCode(USER_CODE_FLASH) && !checkUserCode(USER_CODE_RAM))
        								|| buttonStatusHigh;
  int delay_count = 0;


  while ((delay_count++ < BOOTLOADER_WAIT) || no_user_jump) {
    strobePin(LED_BANK,LED,1,BLINK_SLOW);

  #ifndef STM32F2
    if (dfuUploadStarted()) {
      dfuFinishUpload(); // systemHardReset from DFU once done
    }
  #endif
  }
#endif

	if (checkUserCode(USER_CODE_RAM)) {
    jumpToUser(USER_CODE_RAM);
  } else if (checkUserCode(USER_CODE_FLASH)) {
    jumpToUser(USER_CODE_FLASH);
  } else {
    // some sort of fault occurred, hard reset
    strobePin(LED_BANK,LED,5,BLINK_FAST);
    systemHardReset();
  }

  return 0;               /* can't happen, but placate the compiler */
}
