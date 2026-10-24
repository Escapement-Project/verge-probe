MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x9D800
    RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x40000
}
SECTIONS
{
    .interrupts 0x0 : { KEEP(*(.interrupts)) } > FLASH
    .text 0x200 : { *(.text*) *(.rodata*) } > FLASH
    .data : { *(.data*) } > RAM AT> FLASH
    .bss (NOLOAD) : { *(.bss*) } > RAM
    .core1_code 0x40000 : { KEEP(*(.core1_code)) } > FLASH
}
