# Board-specific configuration values.  Flash and SRAM sizes in bytes.

ifeq ($(BOARD), maple)
   MCU := STM32F103RB
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOA
   ERROR_LED_PIN  := 5
   DENSITY := STM32_MEDIUM_DENSITY
   FLASH_SIZE := 131072
   SRAM_SIZE := 20480
endif

ifeq ($(BOARD), maple_native)
   MCU := STM32F103ZE
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOC
   ERROR_LED_PIN  := 15
   DENSITY := STM32_HIGH_DENSITY
   FLASH_SIZE := 524288
   SRAM_SIZE := 65536
endif

ifeq ($(BOARD), maple_mini)
   MCU := STM32F103CB
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOC
   ERROR_LED_PIN  := 1
   DENSITY := STM32_MEDIUM_DENSITY
   FLASH_SIZE := 131072
   SRAM_SIZE := 20480
endif

ifeq ($(BOARD), maple_RET6)
   MCU := STM32F103RE
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOA
   ERROR_LED_PIN := 5
   DENSITY := STM32_HIGH_DENSITY
   FLASH_SIZE := 524288
   SRAM_SIZE := 65536
endif

ifeq ($(BOARD), aeroquad32f1)
   MCU := STM32F103VE
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOE
   ERROR_LED_PIN  := 5
   DENSITY := STM32_HIGH_DENSITY
   FLASH_SIZE := 524288
   SRAM_SIZE := 65536
   MCU_FAMILY := STM32F1
endif

ifeq ($(BOARD), aeroquad32)
   MCU := STM32F406VG
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOE
   ERROR_LED_PIN  := 5
   DENSITY := STM32_HIGH_DENSITY
   FLASH_SIZE := 524288
   SRAM_SIZE := 65536
   MCU_FAMILY := STM32F2
endif

ifeq ($(BOARD), aeroquad32mini)
   MCU := STM32F103CB
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOB
   ERROR_LED_PIN  := 1
   DENSITY := STM32_MEDIUM_DENSITY
   FLASH_SIZE := 131072
   SRAM_SIZE := 20480
   MCU_FAMILY := STM32F1
endif

ifeq ($(BOARD), freeflight)
   MCU := STM32F103CB
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOB
   ERROR_LED_PIN  := 3
   DENSITY := STM32_MEDIUM_DENSITY
   FLASH_SIZE := 131072
   SRAM_SIZE := 20480
   MCU_FAMILY := STM32F1
endif

ifeq ($(BOARD), discovery_f4)
   MCU := STM32F406VG
   PRODUCT_ID := 0003
   ERROR_LED_PORT := GPIOD
   ERROR_LED_PIN  := 14
   DENSITY := STM32_HIGH_DENSITY
   FLASH_SIZE := 524288
   SRAM_SIZE := 65536
   MCU_FAMILY := STM32F2
endif


# Memory target-specific configuration values

ifeq ($(MEMORY_TARGET), ram)
   LDSCRIPT := $(BOARD)/ram.ld
   VECT_BASE_ADDR := VECT_TAB_RAM
endif
ifeq ($(MEMORY_TARGET), flash)
   LDSCRIPT := $(BOARD)/flash.ld
   VECT_BASE_ADDR := VECT_TAB_FLASH
endif
ifeq ($(MEMORY_TARGET), jtag)
   LDSCRIPT := $(BOARD)/jtag.ld
   VECT_BASE_ADDR := VECT_TAB_BASE
endif
