#if defined (MCU_STM32F205VE) || defined (MCU_STM32F406VG)
  #include "flash_stm32F2.c"
#else
  #include "flash_stm32F1.c"
#endif

