/* Linker script for STM32F1042C6 */

/* Define memory regions. */
MEMORY
{
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 32K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 6K
}

/* Include the common ld script. */
INCLUDE cortex-m-generic.ld
