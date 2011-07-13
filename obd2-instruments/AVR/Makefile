# AVR-GCC Makefile for the OBD2-instrument package.
#
# This is a cross-compile configuration so it's always going to be a bit
# ad hoc.

# Set the CAN controller by the linking in the device driver. 
CAN_DEV_O = mcp2515.o

.SUFFIXES:	.s .c .o .asm .cof .elf .eep .hex

CC	= avr-gcc
AS	= avr-gcc
RM	= rm -f
RN	= mv
CP      = cp
BIN	= avr-objcopy
OBJCOPY = avr-objcopy
INCDIR	= .

ifeq "$(WINAVR)" "WINAVR"
   $(warning You appear to be using Windows.  My condolences.)
  # Bogus line needed for Windows AVR Studio.  It apparently cannot understand
  # exit codes.
  DONE    = @echo Errors: none
else 
  TTYDEV=/dev/ttyUSB0
  AVRDUDE=avrdude -p m1280 -b 57600 -c arduino -P $(TTYDEV)
  DONE=
  MCU?=atmega1280
  CFLAGS=-O2 -ffunction-sections -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -Wall -Wstrict-prototypes -mmcu=$(MCU)
  CFLAGS+=-I../common/
  CFLAGS+=-DALT_USART=1
  # To generate assembler files e.g. .lst/.map/.hex pass an assembler option
  # CFLAGS+= -Wa,-adhlns=$(<:.c=.lst)
  ASFLAGS=-mmcu=$(MCU) -Wa,-mmcu=$(MCU),-gstabs
  LDFLAGS=-mmcu=$(MCU)
  # LDFLAGS+= -Wl,--gc-sections,-Map=$(<:.o=.map),--cref
endif

# Output format can be S records (Motorola) or ihex (Intel)
ROMFORMAT 	= ihex
EEPROMFORMAT    = ihex              # AVR Studio needs Intel HEX format

%o : %c
	$(CC) -c $(CFLAGS) -I$(INCDIR) $< -o $@

%s : %c
	$(CC) -S $(CFLAGS) -I$(INCDIR) $< -o $@

%o : %s
	$(AS) -x assembler-with-cpp $(ASFLAGS) -I$(INCDIR) -c $< -o $@

%o : %asm
	$(AS) -x assembler-with-cpp $(ASFLAGS) -I$(INCDIR) -c $< -o $@

# Explicitly use a suffix of .elf to distinguish from host environment files.
%elf:	%o
	$(CC) $^ $(LIB) $(LDFLAGS) -o $@

ifeq "$(WINAVR)" "WINAVR"
# AVR Studio needs '.cof' files (.coff when  you only have three letters)
# for debug and simulation.
COFFCONVERT=$(OBJCOPY) --debugging \
	--change-section-address .data-0x800000 \
	--change-section-address .bss-0x800000 \
	--change-section-address .noinit-0x800000 \
	--change-section-address .eeprom-0x810000 
%cof: %elf
	$(COFFCONVERT) -O coff-ext-avr $< $@
endif


# Create flash and EEPROM bin files (ihex, srec) from the elf file
%hex: %elf
	$(BIN) -O $(ROMFORMAT) -R .eeprom $< $@

%eep: %elf
	$(BIN) -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O $(EEPROMFORMAT) $< $(@:.elf=.eep)


.phony:	all clean distclean program verify download tar

VPATH=../common/

all:	obd2-instruments.hex can-control.hex
	$(DONE)
obd2-instruments:	obd2-instruments.hex

#  LDFLAGS += --defsym=firmware_crc=0x4242
obd2-instruments.elf:	main.o obd2.o $(CAN_DEV_O) atmega-dev.o\
  serial-commands.o command-plus.o printf.o serial.o spi-adc.o tach.o pi-loop.o
	$(CC) $^ $(LIB) $(LDFLAGS) -o $@

main.o serial.o obd2.o command.o spi-adc.o pi-loop.o:	vvvvroom.h
func-test.o tach.o: vvvvroom.h
main.s:	main.c vvvvroom.h autocrc.h

autocrc.gen: vvvvroom.hex
	$(BIN) -O $(ROMFORMAT) -R .eeprom $< $@
	./genautocrc $@


# This is specific to a Linux environment.
# A bit ugly because the ports will change, but convenient for me.
download: vvvvroom.hex
	$(AVRDUDE) -v -U flash:w:$<

download-test: func-test.hex
	$(AVRDUDE) -v -U flash:w:$<

can-control.elf: can-control.o atmega-dev.o obd2.o $(CAN_DEV_O)\
   command-plus.o serial.o printf.o

can-control: can-control.hex
	$(AVRDUDE) -U flash:w:$<
can-control.o: command.h
can-control.o command-plus: command.h

obd2.o can-control.o mcp2515.o at90can.o: can.h

at90can.o: MCU=at90can32

verify: vvvvroom.hex
	$(AVRDUDE) -U flash:v:$<
#	avrboot /dev/ttyS0 -file $@ -crc -verify -run

# Copy the program from the board to verify or check the version
extract:
	$(AVRDUDE) -U flash:r:$@:r

# Housekeeping rules
clean:
	$(RM) *.hex *.elf *.lst *.map
	$(RM) *.o *.s
	$(RM) autocrc.gen
distclean: clean
	$(RM) *~

tar: obd2.tgz
obd2.tgz: README Makefile *.[ch]
	tar cfvz $@ $^
