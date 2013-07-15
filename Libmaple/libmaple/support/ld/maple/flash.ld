/*
 * Maple (STM32F103RBT6, medium density) linker script for Flash builds.
 */

/*
 * Define memory spaces.
 */
MEMORY
{
  ram (rwx) : ORIGIN = 0x20000C00, LENGTH = 17K
  rom (rx)  : ORIGIN = 0x08005000, LENGTH = 108K
}

/*
 * Use medium density device vector table
 */
GROUP(libcs3_stm32_med_density.a)

REGION_ALIAS("REGION_TEXT", rom);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", rom);

/*
 * Define the rest of the sections
 */
_FLASH_BUILD = 1;

INCLUDE common.inc
