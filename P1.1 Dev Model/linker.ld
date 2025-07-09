/* P1.0/linker.ld: load at 0x00100000 (1 MiB), flat 32-bit */
OUTPUT_FORMAT("elf32-i386")
ENTRY(protected_mode_start)

SECTIONS {
  /* place .text/.data/.bss starting at 1 MiB so bootloader fits below */
  . = 0x00100000;
  .text   : { *(.text*) }
  .rodata : { *(.rodata*) }
  .data   : { *(.data*) }
  .bss    : { *(.bss*)  *(COMMON) }
}