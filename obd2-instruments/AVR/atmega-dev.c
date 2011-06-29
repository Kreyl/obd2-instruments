/* atmega-dev.c: ATMega AVR microcontroller device setup and interface. */
/*
  AVR ATMega device interface code for the QAR motor controller, OBD2
  CAN reporting and OBD2/CAN gauge interface.

  This file contains the common device setup and operation code that
  would otherwise be duplicated in the motor controller and
  gauge/body interface.

	Written 2010 by Donald Becker and William Carlson.

	The original authors may be reached as
	donald.becker@gmail.com
	Annapolis MD 21403

	This software is released under the Gnu Public License (GPL) v2.
	Commercial licenses are available you do not wish to comply with
	the GPL terms.
	Support information and updates are available from the authors.
*/

#if defined(__AVR_ATmega168__)
#define MEGA168
#define COUGAR
#endif
#if defined(__AVR_ATmega1280__)
#define MEGA1280
#endif
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "vvvvroom.h"
extern int serprintf(const char *format, ...)
	__attribute__ ((format(printf, 1, 2)));;

/* A/D Converter functions. */

/* Our base value for ADCSRA ADC Status Register A.
 * Set the clock prescaler to /128, 125KHz. */
const uint8_t adcsra_base = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

void setup_adc(void)
{
	/* The ADC configuration is set to an ADC clock that allows full
	 * precision, <= 200KHz pg. 280.  With a 16MHz clock, that requires
	 * the maximum prescale divisor, 128, for an ADC clock of 125KHz.
	 *
	 * Since a full 10 bit precision conversion takes slightly over 13 ADC
	 * cycles, we get only a 9.6KHz conversion rate.
	 * It is possible to change the prescale divisor, up to 1MHz, if
	 * the full resolution is not needed.  Doubling to 250KHz would have
	 * minimal resolution loss in our electrically noisy environment.
	 */
	ADCSRA = adcsra_base;

	/* Start by doing an immediate first conversion to get the extra-long,
	 * inaccurate one out of the way.  Ideally we would do a conversion of
	 * an internal test souce to verify operation, but we simplify to
	 * use our standard function.
	 */
	start_adc_conversion(0);
	read_adc_voltage();
	return;
}

/*
 * Start a single ADC conversion.
 * The A/D converter reference voltage and channel are set here.
 * This is set to External, using the +5V Vcc as the analog reference.
 * The 5V supply is less accurate than the internal references, but more
 * appropriate for the ratiometric throttle and output current monitoring.
 * This code now allows -1 and -2 to select the internal 0V and 1.1V
 * test sources.  The latter is useful to checking the operating voltage.
 */
void start_adc_conversion(int8_t channel)
{
#if defined(MEGA1280)
	if (channel > 7) {			/* Select the high ADC channels. */
		ADCSRB = (1 << MUX5);
		ADMUX = ADMUX = (01 << REFS0) | (channel & 0x07);
	} else {
		ADCSRB = 0;
		ADMUX = ADMUX = (01 << REFS0) | (channel & 0x1F);
	}
#else
#warning Using the 8 channel A/D converter code.
	ADMUX = ADMUX = (01 << REFS0) | (channel & 0x07);
#endif
	/* Start a one-shop ADC conversion pg. 277 */
	ADCSRA = adcsra_base | (1 << ADSC);
}

/* Read the results of the most recent conversion. */
int read_adc_voltage(void)
{
	while (ADCSRA & (1 << ADSC))
		;
	/* Use the assembler short-cut to read the register pair in order. */
	return ADC;
}

/* Establish a baseline value for measured operating conditions by averaging
 * over 16 samples. */
uint16_t adc_baseline(int8_t channel)
{
	unsigned char i;
	unsigned sum = 0;

	for (i = 16; i > 0; i--) {
		start_adc_conversion(channel);
		wdt_reset();
		sum += read_adc_voltage();
	}
	return sum >> 4;
}

/* Show the voltages on the internal ADC.
 * The older version was strictly for start-up debugging.  It triggered the
 * ADC conversion and thus could only be called before the timer was running
 * and triggering its own A/D conversions.
 * This version only prints the values from raw_adc[].
 */
void show_adc_voltages(void)
{
	int8_t i;
	int count;
	uint16_t voltage;

	/* Start by converting the internal 1.1V source and calculating VCC. */
	count = adc_baseline(-2);

	serprintf(PSTR("System voltages\n"));
	voltage = 1125000L / (long) count;	/* Round a bit to allow 5.000V  */
	serprintf(PSTR("AVR Vcc %d.%3dV (%5d)\n"),
			  voltage/1000, voltage%1000, count);

	for (i = -2; i < 16; i++) {
		start_adc_conversion(i);
		wdt_reset();
		count = read_adc_voltage();
		voltage = (625*(long)count) >> 7;
		serprintf(PSTR("AVR ADC%2d %4d %d.%3dV\n"), i, count,
				  voltage/1000, voltage%1000);
	}
	return;
}


/*
 * Local variables:
 *  compile-command: "make -k"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
