#ifndef _VVVVROOM_H
#define _VVVVROOM_H
/* vvvvroom.h: Vvvvroom motor controller definition file. */
/*
	$Revision: 1.0 $ $Date: 2011/1/22 00:00:21 $
	Hardware compatibility settings.
	This file provides symbolic names and macros to mask the difference
	between hardware implementations.
	It is designed primarily to allow a single source base to support
	several different motor controller designs, instrument displays,
	and microcontroller chips.

	Written 2010-2011 by Donald Becker, William Carlson and Hugo Becker.
	This software may be used and distributed according to the terms
	of the GNU General Public License (GPL), incorporated herein by
	reference.  Firmware interacting with these functions are derivative
	works and thus are covered the GPL.  They must include an explicit
	GPL notice and follow the terms of the license.

	Other contributers:
	<currently none>
*/

#pragma std_sdcc99 

#ifndef tach_get_QRPM
#define tach_get_QRPM() 1234*4
#endif

/* Parameters for the 5V reference 10-bit PIC A/D converter. */
#define ADC_VOLTAGE_SCALE 625
#define NUM_ADC_CHANNELS 8
extern volatile uint16_t raw_adc[NUM_ADC_CHANNELS];

extern struct fault_info {
	volatile unsigned char bits;
	unsigned cycles_off;		/* PWM cycles with 0 output cause by faults. */
	unsigned pi_overwork;
	unsigned motor_overspeed_fault_count;
} fault;


extern unsigned tach_redline_qrpm;			/* Redline in QRPM */
extern unsigned tach_rpm_min_period;		/* Tach period at redline  */
extern unsigned tach_target_qrpm;			/* Target speed in QRPM */

extern uint16_t m_current_peak;
extern uint16_t use_direct_pwm;
extern int current_fb;			/* Current feedback, actual current. */
extern volatile uint32_t clock_1msec;

/* Throttle settings grouped in one spot. */
struct throttle_params {
	uint8_t fault_counts;
	int16_t range;				/* Max - min pre-calculated.  */
	int16_t gain;				/* Pre-calculated multiplier for 0..32K.  */
	uint16_t closed;			/* The idle/foot-off/closed/baseline voltage */
	uint16_t low_fail, high_fail;
	uint8_t  remote;			/* Commanded remote throttle. */
	uint8_t  remote_valid;
	uint16_t remote_time;		/* Timestamp that .remote was set. */
} extern throttle;

/* This state structure is pretty much unchanged from the Cougar
 * controller so that we can act as an add-on to its firmware.
 */
typedef struct {
	int throttle_ref;
	int current_ref;
	int current_fb;
	unsigned raw_hs_temp;
	unsigned raw_throttle;
	unsigned battery_amps;
	uint32_t battery_ah;
} realtime_data_type;

extern realtime_data_type rt_data;

#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
