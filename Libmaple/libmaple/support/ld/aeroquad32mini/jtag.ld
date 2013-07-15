/*
 * Maple Mini (STM32F103CBT6, medium density) linker script for JTAG
 * (bare metal, no bootloader) builds.
 */

/*
 * Define memory spaces.
 */
MEMORY
{
  ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  rom (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
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
