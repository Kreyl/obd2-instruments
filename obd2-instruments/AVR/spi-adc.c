/* spi-adc.c: SPI A/D converter module for the VVVVroom motor controller. */
/*
 * AVR SPI ADC code.
 * Read a Microchip MCP3000 series A/D converter over the SPI bus
 * on an AVR controller.
 *
 */
static const char versionA[] =
"spi-adc: $ Date: $ Copyright Donald Becker\n";

#if defined(__AVR_ATmega168__)
#define MEGA168
#endif
#if defined(__AVR_ATmega1280__)
#define MEGA1280
#endif

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(MEGA168)
#include <avr/iom168.h>
#else
#include <avr/iom1280.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vvvvroom.h"

extern int serprintf(const char *format, ...)
	__attribute__ ((format(printf, 1, 2)));;


/* Some symbolic constants. */

int ADC_enabled = 0;

/* Sending a byte on SPI is easy: write, check status for completion,
 * and read the byte simultaneously clocked in.  This is the generic way to
 * do this, but we always try to do other work while the hardware clocks out
 * the bits so this is rarely used.
 */
static inline char SPI_Transmit(char cData)
{
   SPDR = cData;
   while(!(SPSR & (1<<SPIF)))
      ;
   return SPDR;
} 

/* A variation of SPI_Transmit that just waits for completion.  Use this
 * when the next master-out byte can be prepared before starting to busy-wait.
 */
static inline char SPI_Read(void)
{
   while(!(SPSR & (1<<SPIF)))
      ;
   return SPDR;
} 

/* Initialize the SPI interface.
 * This might be Mega 1280 specific -- I will generalize when it has been
 * tested.
 *  					SS SCK MOSI MISO
 *  AT90USB82/162 		PB0	PB1 PB2 PB3
 *  ATmega48/88/168/328 PB2	PB5 PB3 PB4
 *  ATtiny8			 	-	- 	PA6	PA5
 * We set these SPI pins on port B as outputs:  MOSI SCK SS
 *
 * The MCP8003 series parts have a max 3.7MHz clock at 5V.  That is temptingly
 * close to 4MHz, but we play it safe with 2MHz.  That allows us to use the
 * same clock with the MCP3200 and MCP3300 (2MHz) series.
 */

#define PB_ADC_CS 0			/* Abitrary digital pin used for active-low /CS */
#define ADC_CS_ENABLE	PORTB &= ~(1 << PB_ADC_CS);		/* Chip select low */
#define ADC_CS_DISABLE	PORTB |= 1 << PB_ADC_CS;		/* Chip select high */

static inline void SPI_init(void)
{
	DDRB |= ((1<<DDB2)|(1<<DDB1)|(1<<DDB0));
	/* Turn on: SPI enable, MSB first, 0,0, Master, F_OSC/16. */
	SPCR = ((1 << SPE) | (1 << MSTR) | (1 << SPR0));
	SPSR = 1<<SPI2X;				/* Double clock to 2MHz */
	return;
}


/* Do a full 10 bit conversion.  The timing awkwardly requires 17 bits.
 * We delay the start bit so that the result is LSB aligned.
 */
int adc_convert(char channel)
{
	unsigned char cmd;
	int convert;

	ADC_CS_ENABLE;
	SPDR = 0x01;				/* Send start marker */
	cmd = 0x80 | (channel << 4);
	SPI_Read();					/* Discard bits from sending start marker. */
	/* Start single channel conversion and read back the two high bits. */
	convert = SPI_Transmit(cmd);
	SPDR = 0xFF;				/* And start reading the low bits. */
	convert &= 0x03;			/* While we start packing the result.  */
	convert <<= 8;
	convert |= SPI_Read();
	ADC_CS_DISABLE;
	return convert;
}

/* Do a 9 bit conversion.  This discards resolution to speed up the
 * conversion.  It works unchanged with 10, 12 and 13 bit parts.
 */
int adc_convert9(char channel)
{
	unsigned char cmd;
	int convert;

	cmd = 0xC0 | (channel << 3);
	ADC_CS_ENABLE;
	SPDR = cmd;
	convert = SPI_Read();
	SPDR = 0xFF;				/* Finish reading the low 8 bits. */
	convert &= 0x01;
	convert <<= 8;
	convert |= SPI_Read();
	ADC_CS_DISABLE;
	return convert;
}

/* Do an conversion, returning the result as a signed binary fraction.
 * The MSB of the conversion is the 15th bit of the returned value.
 * This allows using the same calculation with any resolution device,
 * although the caller is responsible for truncating appropriately.
 * This code also allows extending the sample period for better accuracy with
 * high impedance sources.
 */
unsigned int adc_convert_fract(char channel)
{
	unsigned char cmd;
	int convert;

	cmd = 0x18 | (channel);
	ADC_CS_ENABLE;
	SPDR = cmd;
	SPI_Read();
	/* Insert a sample delay here if needed. */
	SPDR = 0xFF;				/* Read the high 7 bits. */
	cmd = SPI_Read();
	SPDR = 0xFF;				/* Read the remaining low bits. */
	convert = (cmd & 0x7F) << 8;
	convert |= SPI_Read();		/* Should clear the LSBs... */
	ADC_CS_DISABLE;
	return convert;
}

int adc_setup(void)
{
	int i;

	serprintf(PSTR("ADC SPI init\n"));

	SPI_init();
	ADC_enabled = 1;

#if 1
	for (i = 0; i < 8; i++) {
		unsigned int test_convert, voltage;
		test_convert = adc_convert(i);
		voltage = (625*(long)test_convert) >> 7;
		serprintf(PSTR("SPI ADC Channel %d %4x 0.000V\n"), i, test_convert,
				  voltage/1000, voltage%1000);
	}
	for (i = 0; i < 8; i++) {
		unsigned int test_convert, voltage;
		test_convert = adc_convert9(i);
		voltage = (625*(long)test_convert) >> 6;
		serprintf(PSTR("SPI ADC Channel %d %4x 0.000V\n"), i, test_convert,
				  voltage/1000, voltage%1000);
	}
	for (i = 0; i < 8; i++) {
		unsigned int test_convert, voltage;
		test_convert = adc_convert_fract(i);
		voltage = (625*(long)test_convert) >> 11;
		serprintf(PSTR("SPI ADC Channel %d %4x 0.000V\n"), i, test_convert,
				  voltage/1000, voltage%1000);
	}
#endif
	return 0;
}


/*
 * Local variables:
 *  compile-command: "make spi-adc.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
