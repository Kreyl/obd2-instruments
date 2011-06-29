/* tach.c: Tachometer code for the VVVVroom motor controller. */
/*
 * This file has the code for interpreting tachometer pulses and
 * maintaining a filtered RPM state variable.
 *
 * This code is simple enough to incorporate into the interrupt handler
 * but is maintained in a separate file to decouple development
 * modifications and faciliate its possible use in the Cougar code.
 *
 * The basic version uses the 8KHz PWM/sample clock to monitor pin level
 * transitions.  This loses resolution compared to an interrupt or
 * hardware timer based approach, but is still quite accurate.  A single
 * sample is better than 1% below 4800RPM (80 rev/sec).  Simple
 * filtering can increase this resolution and increase feedback
 * stability with edge detection noise.
 *
 * One caution is that the state variable must always be used with the
 * 'valid' time.  Zero RPM operation provides no opportunity to update
 * the state variable.
 *
 * The advanced version uses a hardware timer and ICR (input capture
 * register) to get a very precise timestamp for the edge detection.
 * Absurdly precise considering the noise environment.
 *
 * References for understanding this code
 *  None
 *
 * Notes:
 *
 */
static const char versionA[] =
"tach.c: $Version: $ $Date: $ Copyright Donald Becker\n";

#define TACH24 1

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
#include <stdlib.h>
#include <string.h>

#include "vvvvroom.h"

/* Our primary output: tachometer state in Quarter Rotations Per Minute.
 * The reporting range is [0 .. 16,383.75] RPM.
 * This was selected to match the OBD2 resolution, our primary reporting
 * reporting mode.
*/
uint16_t tach_QRPM = 0;
uint32_t tach_last_sample = 0;		/* 32 bit concat of counter_1k+time_high. */

unsigned tach_redline_qrpm;			/* Redline in QRPM */
unsigned tach_rpm_min_period;		/* Tach period at redline  */
unsigned tach_target_qrpm;			/* Target speed in QRPM */

/* Time stamps of recent tachometer edges. */
#if 0
static unsigned tach_ticks[8];
#endif
static uint16_t tach_ticks_1k;
static uint16_t tach_ticks_8k;

/* Called for each edge transition with local 8KHz timer count and
 * the new level (redundant).
 * We use three modes for calculating the speed:
 *  Very slow: clamp to zero and check for absurd numbers
 *  Slow: single sample for quicker response
 *  Normal: filtered sample
 */
void tach_tick(uint8_t counter_8k, uint8_t edge)
{
	uint32_t this_sample = ((uint32_t)time_high<<16) | clock_1msec;
	uint16_t period_1k = clock_1msec - tach_ticks_1k;

	if (period_1k < 32) {
		uint8_t period_8k = counter_8k - tach_ticks_8k;
		/* We are above 1800RPM -- the 8K timer is usable, and more
		 * precise. */
		int speed = 60U * 976U / period_8k;
		speed <<= 3;
	} else if (period_1k < 16000) {
		int speed = 60U * 976U / (clock_1msec - tach_ticks_1k);
		speed = (speed << 2) + speed; 			/* *5 */
	} else {
		/* Less than 6 RPM is pretty much stopped.
		 * Avoid aliasing on the next sample. */
		if (tach_QRPM < 25) {
			uint32_t period = this_sample - tach_last_sample;
			if (period > 10000) {
				tach_QRPM = 0;
			} else {
				tach_QRPM = 1/period;
			}
		}
	}
	tach_ticks_1k = clock_1msec;
	tach_ticks_8k = counter_8k;
	return;
}

/* A tachometer using the hardware input capture facility.
 *
 * We configure a timer as a maximal-count timer running at a high clock
 * rate (62.5KHz or 2MHz) and use the ICR (Input Capture Register) to
 * get a timestamp and raise an interrupt on a signal edge.
 *
 * On the Mega 1280 board we have only ICP4 (JP2-49) and ICP5 (JP2-48)
 * connected, so we are limited to using these timers.
 *
 * We enable the ICR noise filter, which requires four cycles at the new
 * level before triggering the capture.
 *
 * With a 2MHz counter clock, the resolution at high RPMs is quite good
 * 7325 ticks 65529 16382.25 RPM
 * 7324 ticks 65520 16380.0 RPM
 * 19999 ticks 24001 6000.25 RPM
 * 20000 ticks 24000 6000.0 RPM
 * The down-side is that we need at least 24 bit math, which takes significant
 * time.
 *
 * We can still get good resolution with a 16MHz/256 -> 62.5KHz clock.
 * 625 ticks 6000 RPM
 * 626 ticks 23961 QRPM 5990.25 RPM
 * This allows us to use 16 bit math and a still get minimum 57 RPM before
 * overflow.
 */
#define TIMER_MODE 0
#define F_CPU 16000000			/* Our 16MHz clock. */
#if defined(TACH24)
#define TIMER_PRESCALE (2 << CS50)	/* Index: 0,1,8,64,256,1024,TnUp,TnDown */
#elif defined(TACH16)
#define TIMER_PRESCALE (5 << CS50)	/* Index: 0,1,8,64,256,1024,TnUp,TnDown */
#else
#warning No tachometer resolution defined, using 16 bits.
#define TACH16
#define TIMER_PRESCALE (5 << CS50)	/* Index: 0,1,8,64,256,1024,TnUp,TnDown */
#endif

/* Our internal state as 16+8 bit timestamps.
 * The timer counts the 16 low bits, while the software handles the
 * overflow into the upper 8 bits.
 */
static uint8_t tach_period_overflow; 	/* Start at zero RPM. */
static uint16_t tach_last_triggered;
uint8_t tach_last_period_high;
uint16_t tach_last_period;

#if defined(TACH16)
uint8_t tach_icp_interrupts = 0;
uint16_t tach_ovf_interrupts = 0;
#elif defined(TACH24)
long tach_icp_interrupts = 0;
long tach_ovf_interrupts = 0;
#endif

void setup_tach(void)
{
	/* No need to stop or clear the counter. */
	/* No output pins used, leaving them available for other purposes. */
	TCCR5A = ((TIMER_MODE & 0x3) << WGM50);
	/* Set pre-scaler to /8, turn on noise filter set upper mode bits. */
	TCCR5B =  TIMER_PRESCALE | ((TIMER_MODE & 0xC) << (WGM52 - 2))
		| (1 << ICNC5);
	/* On the Mega we use PL1/ICP5 on JP2-48 as the input, with PL0/ICP4
	 * as an alternate.  Enable both pins as input with pull-up. */
	PORTL = 0x03;
	DDRL = 0x00;
	TIMSK5 = 0x21;					/* Interrupt on capture and overflow. */
	tach_period_overflow = 128; 		/* Start at zero RPM. */
	return;
}

/* The input cature happens on a falling edge at the ICP input.
 * This interrupt is a higher priority than the overflow interrupt so we
 * need not worry about the variables changing while we are calculating.
 */
ISR(SIG_INPUT_CAPTURE5)
{
	uint16_t capture_count = ICR5;

	tach_last_period = capture_count - tach_last_triggered;
	tach_last_period_high = tach_period_overflow;
	if (capture_count < tach_last_triggered)
		tach_last_period_high -= 1;

    tach_period_overflow = 0;
	tach_last_triggered = capture_count;
	tach_icp_interrupts++;
	return;
}

/* The timer overflows about 30Hz or 1Hz (w/ 62.5KHz).
 * We track high bits so that we can track very slow speeds, but there
 * isn't a need to track below about 15 RPM.
 */
ISR(TIMER5_OVF_vect)
{
    if (++tach_period_overflow > 127) {
		tach_period_overflow = 128;
	}
	tach_ovf_interrupts++;
	return;
}

/* Calculating the RPM requires an expensive division. Defer calculating
 * until it is requested, and keep the value around until it changes.
 * Reporting is in quarter turns to match OBD2 reporting.
 * QRPM = 60 seconds * 4 quarter turns * 2MHz clock / period.
 */
static uint32_t tach_recent_report_period = 0;
static uint16_t tach_recent_report_value = 0;

uint16_t tach_get_QRPM(void)
{
	packL_t tach_period;

    if (tach_period_overflow > 127)
		return 0;
	/* If we are slowing, ramp down by reporting the best possible speed if
	 * the trigger happened right now.  It is an bounding estimate, so
	 * precision is pointless. */
	if (tach_period_overflow > tach_last_period_high) {
		return 7325 / (int)tach_period_overflow;
	}

	cli();	{
		tach_period.w[1] = tach_last_period_high;
		tach_period.w[0] = tach_last_period;
	} sei();

	if (tach_period.l != tach_recent_report_period) {
		if (tach_period.l < 7325) 		/* Minimum period at 16,383 QRPM. */
			tach_recent_report_value = 0xFFFF;
		else
			tach_recent_report_value = 60L*4L*2000000L / tach_period.l;
		tach_recent_report_period = tach_period.l;
	}
	return tach_recent_report_value;
}

/*
 * Local variables:
 *  compile-command: "make tach.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
