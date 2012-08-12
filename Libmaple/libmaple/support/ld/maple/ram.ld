/*
 * Maple (STM32F103RBT6, medium density) linker script for RAM builds.
 */

/*
 * Define memory spaces.
 */
MEMORY
{
  ram (rwx) : ORIGIN = 0x20000C00, LENGTH = 17K
  rom (rx)  : ORIGIN = 0x08005000, LENGTH = 0K
}

/*
 * Use medium density device vector table
 */
GROUP(libcs3_stm32_med_density.a)

REGION_ALIAS("REGION_TEXT", ram);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", ram);

/*
 * Define the rest of the sections
 */
INCLUDE common.inc
