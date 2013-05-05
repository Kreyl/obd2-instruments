/* stm32-adc.c: STM32 A/D conversion for the VVVVroom motor controller. */
/*
	Written 2010-2011 by Donald Becker and William Carlson.

	The VVVVroom motor controller firmware rewritten for the STM32 processor.
	This file contains the A/D converter routines.
*/
static const char versionA[] =
"$Id: stm32-adc.c $\n";

#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#include "armduino.h"
#include "vvvvroom.h"
#include "can.h"


extern volatile uint16_t raw_adc[NUM_ADC_CHANNELS];
extern volatile unsigned raw_adc_sample_time[NUM_ADC_CHANNELS];

/* A/D Converter functions. */

/*
* Start a single ADC conversion.
*/
void start_adc_conversion(int8_t channel)
{
	ADC1_SR = 0;
	ADC1_JSQR = (channel < 0 ? 0 : (channel<<15));
	ADC1_CR2 |= ADC_JSWSTART;
}

/* Read the results of the most recent conversion. */
int read_adc_voltage(void)
{
	uint8_t i = 255;
	do {
		if (ADC1_SR & ADC_JEOC)
			return ADC1_JDR1;
	} while (--i > 0);
	ADC1_CR2 |= ADC_JSWSTART | 0xF000 | ADC_ADON;
	/* Give the conversion another 255 loops. */
	while (! (ADC1_SR & ADC_JEOC) && --i > 0)
		;
	/* Return an invalid result rather than hang the system. */
	return ADC1_JDR1 & 0xFFF;
}

/* Configure the A/D converter.
 * Including setting the per-channel sample time.  The sample time is
 * an irregular function of clock cycles (12MHz for us), and
 * is calculated from the source impedance and required accuracy.
 * Channel 0  A1302 sensor     5 ohm,     0=1.5 cycles
 * Channel 16 Internal temp    17.1 usec  7=239.5 cycles (20 usec)
 * For 1/4 LSB resolution at 12 bit (extra precise) use the following
 * Index Sample Sample Input
 *      cycles   uSec  KOhms
 *  0	  1.5	0.125	 0.4
 *  1	  7.5	0.625	 5.9
 *  2	 13.5	1.125	11.4
 *  3	 28.5	2.375	25.2
 *  4	 41.5	3.45	37.2
 *  5	 55.5	4.625	50
 *  6	 71.5	5.96	NA
 *  7	239.5  20		NA
 */

int adc_setup(void)
{

	/* Per-channel sample time. */
	ADC1_SMPR1 = 0x00FC0000;	/* Ch16 and Ch17 get a long sample time. */
	ADC1_SMPR2 = (1 <<27)|( 1 <<24) 				/* ADC9/8 */
		|( 1 <<21)|( 1 <<18)|( 1 <<15)|( 1 <<12)	/* ADC7/6/5/4 */
		|( 1 << 9)|( 7 << 6)|( 1 << 3)|( 1 << 0);	/* ADC3/2/1/0 */
	/* Turn on, then start a calibration cycle. */
	ADC1_CR2 = ADC_TSVREFE | ADC_CAL | ADC_CONT | ADC_ADON;
	ADC1_CR2 = ADC_ADON;

#if defined(USE_ADC_DMA) || 0
	/* Set the regular sequence to 1,2,3..16 */
	ADC1_SQR1 =      (16-1)<<20 | 15<<15 | 14<<10 | 13<<5 | 12<<0;
	ADC1_SQR2 = 11<<25 | 10<<20 |  9<<15 |  8<<10 |  7<<5 |  6<<0;
	ADC1_SQR3 =  5<<25 |  4<<20 |  3<<15 |  2<<10 |  1<<5 |  0<<0;
	/* Configure DMA channel 1, tied to ADC1. */
	DMA_CNDTR1 = 16;			/* Transfer 16 values.  */
	DMA_CPAR1 = ADC1_DR;
	DMA_CMAR1 = (uint32_t)raw_adc;
	/* Medium priority 16 bit transfer, peripheral to memory,
	 * circular mode with memory increment.
	 */
	DMA_CCR1 = 0x15A1;
#endif

	return 0;
}

/* Establish a baseline value for measured operating conditions by averaging
 * over 16 samples.
 * Reduce the sample count if the A/D converter has more than 12 bits. */
unsigned int adc_baseline(int8_t channel)
{
	char i;
	uint16_t sum = 0;

	for (i = 16; i > 0; i--) {
		start_adc_conversion(channel);
		wdt_reset();
		sum += read_adc_voltage();
	}
	return sum >> 4;
}

#if 0
/* Show the voltages on the internal ADC.
 * Note that this is strictly start-up debugging.  It can only be called
 * before the timer is triggering A/D conversions.
 */
void show_adc_voltages(void)
{
	int8_t i;
	uint16_t count;
	uint16_t voltage;

	/* Throw away the first conversions. */
	adc_baseline(16);
	/* Start by converting the internal 1.1V source and calculating VCC. */
	count = adc_baseline(17);

	serprintf(PSTR("System voltages\r\n"));
	voltage = 4096*1200L / (uint32_t) count;
	serprintf(PSTR("STM Vcc %d.%3dV (%4d)\r\n"),
			  voltage / 1000, voltage % 1000, count);

	for (i = 0; i < 18; i++) {
		start_adc_conversion(i);
		wdt_reset();
		count = read_adc_voltage();
		raw_adc[i] = count;
		voltage = (13*count) >> 4;
		serprintf(PSTR("STM ADC%2d %4d %d.%3dV\r\n"),
				  i, count, voltage / 1000, voltage % 1000);
	}
	return;
}
#endif

/* Print the values in adc_raw[] as voltages.
 * In normal operation the control loop updates the raw_adc[] values.
 * If it's not running gather_adc_voltages() should be called to fill
 * the array.
 */
void show_adc_voltages(void)
{
	int i;

	serprintf(PSTR("System voltages\r\n"));
	serprintf(PSTR("STM Vcc %4dmV (%4d)\r\n"),
			  4096*1200 / (uint32_t) raw_adc[17], raw_adc[17]);

	for (i = 0; i < NUM_ADC_CHANNELS; i++) {
		uint16_t voltage = (13*raw_adc[i]) >> 4;
		serprintf(PSTR("STM ADC%2d %4d %d.%3dV\n"),
				  i, raw_adc[i], voltage / 1000, voltage % 1000);
	}
	return;
}
void gather_adc_voltages(void)
{
	int i;

	/* Assume both the internal 1.2V source and temp sensor are powered. */
	for (i = 0; i < 18; i++) {
		start_adc_conversion(i);
		wdt_reset();
		raw_adc[i] = read_adc_voltage();
	}
	return;
}


/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
