#!/usr/bin/make
# PIC build environment for OBD2 / CAN instrument code
# Written 2011 by Donald Becker and Hugo Becker
# Released under the GPL v2

CC=sdcc
LD=gplink
ASM=gpasm
AR=gplib			# Not quite 'ar' compatible

#CFLAGS= -I/usr/share/sdcc/lib/pic16 
CFLAGS+= -mpic16 -p18f2480 -I. -I../common/
LDFLAGS= -s /usr/share/gputils/lkr/18f2480.lkr
# LDFLAGS= 

VPATH=../common/

%.lst: %.c
	$(CC) $(CFLAGS) -c $< -Wa,-adhlns=$(<:.c=.lst)
%.elf: %.o
	$(LD) $(LDFLAGS) $< -o $@
%.hex: %.o
	$(LD) $(LDFLAGS) $< -o $@
%.bin: %.elf
	$(OBJCOPY) $< -O binary $@

%.o: %.c
	$(CC) -c $(CFLAGS) $<
%.o: %.asm
	gpasm -c $(CFLAGS) $<

.phony:	all clean distclean program verify download tar

all: revolt-lcd.hex can-control.hex cougarlcd.hex

revolt-lcd.hex: lcd.o eCAN.o pic-dev.o revolt.o
	$(CC) -mpic16 -p18f2480 -o $@ -m $^

can-control.hex: can-control.o printf.o command-plus.o eCAN.o pic-dev.o
	$(CC) -mpic16 -p18f2480 -o $@ -m $^

can-control-2515.hex: can-control.o printf.o command-plus.o mcp2515.o pic-dev.o
	$(CC) -mpic16 -p18f2480 -o $@ -m $^

# The basic blink-LED test program
blinky.hex: blinky.o
	$(CC) $(CFLAGS) -o $@ -m $^ /usr/share/sdcc/lib/pic16/libdev18f2480.lib

clean:
	rm -f *.o *.cof *.elf *.bin *.lst *.cod *.asm
distclean: clean
	rm -f *~ *.hex
