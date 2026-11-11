/* AetherVNA firmware: AT32F403ACGT7 behind Zeenko's bootloader.
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
  .noinit (NOLOAD) : { *(.noinit*) . = ALIGN(4); _end_static = .; } > ram
  _estack = ORIGIN(ram) + LENGTH(ram);
  ASSERT(_estack - _end_static >= 8K, "less than 8 KB of RAM left for the stack")
  /* Saved settings and calibrations live at 0x08050000-0x0807FFFF (store.c). */
  _image_end = LOADADDR(.data) + SIZEOF(.data);
  ASSERT(_image_end <= 0x08050000, "the image would overlap the saved settings")
}
