#!/bin/sh
# Rebuilds layout.elf and the S-record objcopy makes of it, the expected value for the ELF reader.
set -eu
cd "$(dirname "$0")"
arm-none-eabi-gcc -mcpu=cortex-m33 -mthumb -nostdlib -nostartfiles -Wl,-T,layout.ld -Wl,--build-id=none -o layout.elf layout.S
arm-none-eabi-objcopy -O srec layout.elf layout.s19
