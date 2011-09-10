#!/usr/bin/make
# ARM build environment for Qar code
# Written 2010/2011 by WWC and DB
# Released under the GPL v2

# Commonly modified settings

CAN_O = bxCAN.o  # Or do  make CAN_O=mcp2515.o 

# Chip support environment and ST-Link tools are available from
#  http://code.google.com/p/arm-utilties
STLINK-DOWNLOAD=/usr/bin/stlink-download
ARM-UTILS=../armduino

ifneq "$(wildcard /usr/bin/arm-none-eabi-gcc)" ""
ARM=arm-none-eabi
else
ARM=arm-linux-gnueabi
endif

CC=$(ARM)-gcc
LD=$(ARM)-ld
ARM-CC=$(ARM)-gcc
ARM-LD=$(ARM)-ld
ARM-OBJCOPY=$(ARM)-objcopy
ARM-OBJDUMP=$(ARM)-objdump

ASFLAGS= -g
LDFLAGS= -T$(ARM-UTILS)/stm32.ld -nostartfiles
#LDFLAGS=-Tstm32f100.ld -L../lib  $(ARM-UTILS)/printf.o
# Super sleazy way to add the printf library.
LDEXTRAS= $(ARM-UTILS)/printf.o
CFLAGS = -fno-common -fconserve-stack -fcaller-saves -mcpu=cortex-m3 -mthumb -I.
CFLAGS += -g -O1 -Wall -Wstrict-prototypes
CFLAGS += -I$(ARM-UTILS)/ -I../common/ -DSTM32=1

%.lst: %.c
	$(ARM-CC) $(CFLAGS) -c $< -Wa,-adhlns=$(<:.c=.lst)
%.elf: %.o
	$(ARM-LD) $(LDFLAGS) $< -o $@
%.bin: %.elf
	$(ARM-OBJCOPY) $< -O binary $@

# Other interesting options:     -ffreestanding -nostdlib -nostdinc

# Disassemble
ARM-DISASM=$(ARM-OBJDUMP)  -marm -Mthumb -EL -b binary -D

VPATH=../common/

all: qar-control.bin can-control.bin bx-can-control.bin mcp-can.bin

download: qar-control.bin
	$(STLINK-DOWNLOAD) /dev/stlink erase flash:w:$<
erase: 
	$(STLINK-DOWNLOAD) /dev/stlink debug reset erase=all

qar-control.elf: vvvv-main.o stm32-devs.o stm32-adc.o pi-loop.o \
  obd2.o $(CAN_O) serial.o motor-commands.o command-plus.o tach.o
	$(ARM-LD) $(LDFLAGS) -o $@ $^  $(LDEXTRAS)

can-control.elf: can-control.o stm32-devs.o stm32-adc.o \
  serial.o command-plus.o obd2.o $(CAN_O)
	$(ARM-LD) $(LDFLAGS) -o $@ $^  $(LDEXTRAS)
bx-can-control.elf: can-control.o stm32-devs.o stm32-adc.o \
   serial.o command-plus.o obd2.o bxCAN.o
	$(ARM-LD) $(LDFLAGS) -o $@ $^ $(LDEXTRAS)
mcp-can.elf: can-control.o stm32-devs.o stm32-adc.o \
   serial.o command-plus.o obd2.o mcp2515.o
	$(ARM-LD) $(LDFLAGS) -o $@ $^ $(LDEXTRAS)

can-control: can-control.bin
	$(STLINK-DOWNLOAD) /dev/stlink erase flash:w:$<

mcp-can: mcp-can.bin
	$(STLINK-DOWNLOAD) /dev/stlink erase flash:w:$<

vvvv-main.o stm32-devs.o stm32-adc.o obd2.o pi-loop.o tach.o: vvvvroom.h
can-control.o motor-commands.o command-plus.o: command.h
can-control.o obd2.o mcp2515.o bxCAN.o: can.h

clean:
	rm -f *.o *.elf *.bin
distclean: clean
	rm -f *~
