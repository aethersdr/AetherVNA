/* LiteVNA probe firmware: AT32F403ACGT7 behind Zeenko's bootloader.
 * The bootloader owns 0x08000000-0x08003FFF and starts the application at
 * 0x08004000. RAM is limited to the 96 KB every AT32F403A has, whatever the
 * option bytes extend it to. SPDX-License-Identifier: GPL-3.0-or-later */
MEMORY
{
  flash (rx)  : ORIGIN = 0x08004000, LENGTH = 1008K
  ram   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}
ENTRY(reset_handler)
SECTIONS
{
  .text : {
    KEEP(*(.vectors))
    *(.text*) *(.rodata*)
    . = ALIGN(4);
  } > flash
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > ram AT > flash
  .bss (NOLOAD) : { _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > ram
  _estack = ORIGIN(ram) + LENGTH(ram);
}
