# LiteVNA probe firmware. SPDX-License-Identifier: GPL-3.0-or-later
CC = arm-none-eabi-gcc
CFLAGS = -mcpu=cortex-m4 -mthumb -Os -ffreestanding -nostdlib -Wall -Wextra -Werror -fno-builtin
probe.bin: probe.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size $<
probe.elf: probe.c link.ld
	$(CC) $(CFLAGS) -T link.ld -o $@ probe.c -lgcc
clean:
	rm -f probe.elf probe.bin
