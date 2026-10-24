/* A stand-in for the firmware's layout: vectors at 0, code at 0x200, .data loaded after the code
 * and run from RAM, and a second region at 0x40000 where the firmware keeps CPU 1's image. */
    .syntax unified
    .thumb

    .section .interrupts, "a"
    .word 0x20010000
    .word reset + 1
    .word 0, 0

    .section .text, "ax"
    .thumb_func
reset:
    ldr r0, =counter
    ldr r1, [r0]
    adds r1, #1
    str r1, [r0]
    b reset

    .section .data, "aw"
counter:
    .word 0x11223344, 0x55667788, 0x99aabbcc

    .section .core1_code, "a"
    .fill 600, 1, 0xc1

    .section .bss, "aw", %nobits
    .space 64
