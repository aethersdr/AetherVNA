# AetherVNA firmware. SPDX-License-Identifier: GPL-3.0-or-later
CC = arm-none-eabi-gcc
GIT_VERSION := $(shell git describe --always --dirty --tags 2>/dev/null || echo unknown)
CFLAGS = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -ffreestanding -nostdlib \
         -Wall -Wextra -Werror -fno-builtin -fsingle-precision-constant -DGIT_VERSION='"$(GIT_VERSION)"'
SRC = main.c gpio.c usb.c console.c vna.c ui.c lcd.c sd.c fonts.c font5x7.c
aethervna.bin: aethervna.elf
	arm-none-eabi-objcopy -O binary $< $@
	arm-none-eabi-size $<
aethervna.elf: $(SRC) hw.h vna.h ui.h lcd.h fonts.h link.ld
	$(CC) $(CFLAGS) -T link.ld -o $@ $(SRC) -lgcc
clean:
	rm -f aethervna.elf aethervna.bin
