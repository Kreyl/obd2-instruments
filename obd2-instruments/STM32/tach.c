/* tach.c: Tachometer code for the VVVVroom motor controller. */
/*
 * This file has the code for interpreting tachometer pulses and
 * maintaining a filtered RPM state variable.
 *
 * It uses a hardware timer and ICR (input capture
 * register) to get a very precise timestamp for the edge detection.
 *
 * References:
 *  None
 *
 * Notes:
 *
 */
static const char versionA[] =
"tach.c: $Version: $ $Date: $ Copyright Donald Becker\n";

/* Set the precision of tach calculations. */
#define TACH24 1

#if defined(STM32)
#include <armduino.h>
#define cli()
#define sei()
#else
#include <stdlib.h>
#include <string.h>
#endif

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

/* A tachometer using the hardware input capture facility.
 *
 * We configure a timer as a maximal-count timer running at a high clock
 * rate (62.5KHz or 2MHz) and use the ICR (Input Capture Register) to
 * get a timestamp and raise an interrupt on a signal edge.
 *
 * With a 2MHz counter clock, the resolution at high RPMs is quite good
 * 7325 ticks 65529 16382.25 RPM
 * 7324 ticks 65520 16380.0 RPM
 * 19999 ticks 24001 6000.25 RPM
 * 20000 ticks 24000 6000.0 RPM
 * The down-side is that we need at least 24 bit math, which takes significant
 * time on an AVR.
 *
 * We can still get good resolution with a 16MHz/256 -> 62.5KHz clock.
 * 625 ticks 6000 RPM
 * 626 ticks 23961 QRPM 5990.25 RPM
 * This allows us to use 16 bit math and a still get minimum 57 RPM before
 * overflow.  While we can spin the motor slower than this, speeds this low
 * are not useful in normal operation.
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
	tach_redline_qrpm = 0;			/* Redline in QRPM */
	tach_target_qrpm = 0;			/* Target speed in QRPM */

	TIM2_CR1 = 0x00;
	TIM2_CR2 = 0x00;
	TIM2_SMCR = 0x0300;
	TIM2_CR1 = 0x01;

	tach_period_overflow = 128; 		/* Start at zero RPM. */
	return;
}

/* The input cature happens on a falling edge at the ICP input.
 * This interrupt is a higher priority than the overflow interrupt so we
 * need not worry about the variables changing while we are calculating.
 */
ISR(TIM2_CC)
{
	uint16_t capture_count = TIM2_CNT;

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
ISR(TIM2_UP)
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
