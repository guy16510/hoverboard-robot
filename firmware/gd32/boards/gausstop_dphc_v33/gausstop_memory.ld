/* SPDX-License-Identifier: GPL-3.0-only */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 8K
}

ENTRY(Reset_Handler)
_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
  .text :
  {
    . = ALIGN(4); *(.text) *(.text*) *(.glue_7) *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .;
  } > FLASH
  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4); _sdata = .; *(.data) *(.data*) . = ALIGN(4); _edata = .;
  } > RAM AT > FLASH
  .bss (NOLOAD) :
  {
    . = ALIGN(4); _sbss = .; __bss_start__ = .; *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4); _ebss = .; __bss_end__ = .;
  } > RAM

  /* 64-byte fixed debugger heartbeat below the reserved 1 KiB stack. */
  .heartbeat 0x20001BC0 (NOLOAD) :
  {
    KEEP(*(.heartbeat))
  } > RAM

  PROVIDE(end = _ebss);
  PROVIDE(_end = _ebss);
  .ARM.attributes 0 : { *(.ARM.attributes) }

  ASSERT(SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.data) <= 64K,
         "flash layout exceeds 64 KiB")
  ASSERT(_ebss <= 0x20001BC0, "static RAM exceeds 7104-byte practical limit")
  ASSERT(SIZEOF(.heartbeat) <= 64, "recovery heartbeat exceeds reservation")
}
