/*
 * Maple Native (STM32F103ZET6, high density) linker script for Flash builds.
 */

MEMORY
{
  ram (rwx) : ORIGIN = 0x20000C00, LENGTH = 61K
  rom (rx)  : ORIGIN = 0x08005000, LENGTH = 492K
}

/*
 * Use high density device vector table
 */
GROUP(libcs3_stm32_high_density.a)

REGION_ALIAS("REGION_TEXT", rom);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", rom);

/* Specify heap boundary addresses on the external SRAM chip */
_lm_heap_start = 0x60000000;
_lm_heap_end = 0x60100000;

_FLASH_BUILD = 1;
INCLUDE common.inc

