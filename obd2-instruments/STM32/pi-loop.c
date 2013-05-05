/* pi-loop.c: Proportional-integral loop for the VVVVroom motor controller. */
/*
 * The motor control Proportional-Integral loop code.
 * This version is based on the PI loop from the Open ReVolt Cougar firmware,
 * released under the GPL.  That allows us to start out with their tested
 * feedback parameters.
 *
 * This code implements the proportional-integral loop to set the motor
 * PWM control.  A PI control loop is a feedback controller which drives
 * the motor with a weighted sum of the error (difference between the output
 * and desired set-point) and the integral of that value.
 *
 * Using the integral term causes the steady-state error to reduce to zero,
 * which is not the case for proportional-only control in general.
 *
 * This loop runs at 4KHz.
 */
static const char versionA[] =
"$Id: pi-loop.c 144 2011-03-23 14:46:55Z becker $ Copyright Donald Becker\n";

#if defined(__AVR_ATmega168__)
#define MEGA168
#endif
#if defined(__AVR_ATmega1280__)
#define MEGA1280
#endif

#if defined(STM32)
#include <armduino.h>

#define set_pwm_width(pwm_width, pwm_sr_width) \
	{ TIM1_CCR1 = TIM3_CCR3 = pwm_width;  TIM3_CCR4 = pwm_width + 37; }
unsigned abs(int);

#else
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(MEGA168)
#include <avr/iom168.h>
#elif defined(__AVR_ATmega1280__)
#include <avr/iom1280.h>
#endif

#if defined(PWM8K)					/* Support possible Cougar users. */
#define	set_pwm_width(pwm_width, pwm_sr_width) \
	/* With an 8KHz pulse, the counter goes twice as high, so we scale up. */
		cli(); OCR3A = pwm_width << 1; OCR3B = pwm_sr_width << 1; sei()
#else
#define	set_pwm_width(pwm_width, pwm_sr_width) \
		do {																\
			cli();														\
			OCR3A = pwm_width;											\
			OCR3C = pwm_width;			/* LED used for bench feedback. */ \
			OCR3B = pwm_sr_width;										\
			sei();														\
		} while(0)
#endif

#include <util/crc16.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#include "vvvvroom.h"

struct throttle_params throttle;

struct pi_state {
	int32_t K1;					/* Scaled Kp (proportional constant) */
	int32_t K2;					/* Scaled Ki (integral feedback constant) */
	int16_t error_old;
	uint32_t pwm;
};
static struct pi_state pi;

/* max current allowed by present temperature. */
unsigned max_current_ref;
int throttle_ref;		  /* Reference (desired) throttle */
int current_ref;		  /* Reference (desired) current */
int current_fb;			  /* Current feedback (actual current) */

/* PWM pecentage calculated by the P-I loop, [0-511]. */
volatile uint16_t pwm_width;
volatile uint16_t pwm_sr_width;		/* Sync rectification setting */

/* Medium-term PWM average. */
static unsigned pwm_avg = 0;
#if defined(USE_32BIT_PWM_AVG)
static int32_t pwm_avg_32 = 0;
#endif

/*
 * We calculate the operatings parameters assuming a 16KHz PWM frequency and
 * a 9 bit (0..511 range) PWM timer.  If the frequency is lowered, the
 * Kp and Ki parameters must be proportionally increased to retain the same
 * response.  If the PWM timer count is increased, the PWM count must be
 * scaled to match.
 */

/* Initialize the P-I state.
 * The parameters are usually read from the EEPROM, but may be set
 * over the serial port or CAN bus.  Or perhaps eventually based on
 * traction response hinting at hill climb/descent.
 * This usually happens before the interrupts are active and does not
 * need to be locked in that case.
 * The caller should disable interrupts if parameters are changed during
 * operation.
 */
void pi_init(void)
{
	pi.K1 = (uint32_t)config.Kp << 10;
	pi.K2 = (uint32_t)config.Ki - pi.K1;

	pi.pwm = 0;
	throttle.fault_counts = 0;
	throttle.range =
		config.throttle_max_raw_counts - config.throttle_min_raw_counts;
	/* Calculate the gain to be < 32768 at full throttle. */
	throttle.gain = 32767 / throttle.range;
	throttle.closed = config.throttle_min_raw_counts;
	throttle.low_fail = config.throttle_fault_raw_counts;
	throttle.high_fail = config.throttle_max_raw_counts + 64;
	/* Inserted for debugging, remove later. */
	serprintf("Throttle range %d..%d, gain %d, low_fail %d high fail %d.\r\n",
			  throttle.closed, config.throttle_max_raw_counts,
			  throttle.gain, throttle.low_fail, throttle.high_fail);
	return;
}

/* PI loop code -- close the feedback loop.
 * The PI constants were copied directly from Cougar, so we kept the
 * cycle-by-cycle structure and repetition rate approximately the same.
 *
 * The loop runs after every motor current sample -- 4KHz / 250 usec.
 * The throttle sample might be up to 14 samples old.
 *
 * It is called in the interrupt handler, just after interrupts are re-enabled.
 * This allows the ISR to re-enter in order to get fresh A/D samples at
 * the regular 8KHz rate.
 *
 * The loop is passed the throttle (Set Point / Target Value) and motor
 * current (Plant Value / Actual Value).  Raw sensor values are passed
 * and normalized inside here since the throttle does not change
 * frequently enough to be re-sampled and re-normalized each cycle.
 */
void pi_loop(unsigned raw_m_current, unsigned raw_throttle)
{
	static uint8_t calc_phase = 0;
	int16_t motor_current;			/* Normalized motor current. */
	uint32_t luv1;

	/* Motor current fault checking and normalization.
	 * If current the current is flowing back (regenerative charging,
	 * throttle-close transient or, more likely, noise/drift) clamp it to 0.
	 * If reverse value is far below the baseline, set a fault. */
	motor_current = baseline_m_current - raw_m_current;
	if (motor_current < 0) {
		/* Motor current fault threshold is -0.5V down or 1024/10 */
		if (raw_m_current < 102)
			fault_bits |= VREF_FAULT;
		current_fb = 0;
	} else {
		/* The motor current should now be in the range [0, 250] or so.
		 * Scale it up to about [0..511] to match the PWM control resolution.
		 * 19/8 is almost 2.4.
		 */
		current_fb = (motor_current * 19) >> 3;
	}

	/* Track the peak observed current. */
	if (motor_current > m_current_peak)
		m_current_peak = motor_current;

	/* If we are in fixed-proportional mode or our target current is
	 * zero, we don't need to do more.
	 * Otherwise calculate the core idea behind P-I, add the averaged error.
	 */
	if (use_direct_pwm) {
		if (fault_bits & ~VREF_FAULT) {
			fault.cycles_off++;
			pwm_width = 0;
		} else if (current_fb > max_current_ref) {
			pwm_width = 0;
		} else {
			pwm_width =
				throttle_ref < use_direct_pwm ? throttle_ref : use_direct_pwm;
		}
	} else if (current_ref == 0) {
		pi.pwm = 0;
		pi.error_old = 0;
		pwm_width = 0;
	} else {
		int16_t pwm_local;
		int16_t m_current_error;

		m_current_error = current_ref - current_fb;
		pi.pwm += (pi.K1 * m_current_error) + (pi.K2 * pi.error_old);
		pi.error_old = m_current_error;

		/* That is it!  We are done with the core PI feedback.  Now we
		 * just scale the result and handle corner cases.
		 * At 99%, the very short off pulse just generates controller
		 * heat without aiding control, so we go full-on and let the
		 * feedback loop switch between e.g. 98% and 100%.
		 * A 1% pulse is useful, but similarly inefficient.  We should
		 * set a pulse width lower bound and add noise to dither the output.
		 */
		pwm_local = pi.pwm >> 16;
		if (pwm_local < 0) {
			pi.pwm = 0L;
			pwm_local = 0;
		} else if (pwm_local > 508) { 		/* Should be a config setting. */
			pi.pwm = (510L << 16);
			pwm_local = 511;
		} else if (pi.pwm & 0x8000)			/* Round up efficiently. */
			pwm_local++;
		pwm_width = pwm_local;				/* Set the volatile only once. */
	}
	/* Hard-clip the output. */
	if (motor_current > config.motor_amps_limit)
		pwm_width = 0;

	/* Calculate the SR gate drive.  No drive at zero speed or max throttle.
	 * Monitor the motor current at 0 throttle and shut off before the
	 * current goes negative to avoid braking/regeneration.
	 */
	if (pwm_width > 0) {
		/* Delay switching on by PWM_SR_DEADTIME.  A setting higher than
		 * TIMER_TOP is OK, just leaving the intrinsic/freewheel/fast diode
		 * to conduct the narrow pulse. */
		pwm_sr_width = pwm_width + PWM_SR_DEADTIME;
	} else {
		/* Regeneration and motor brake logic would go here. */
		/* For a brief period after throttle off we have freewheeling current.
		 * Leave the freewheel device on until we are close to the zero
		 * current crossing, then let the uncontrolled device take over. */
		if (motor_current > 10)			/* High-side conducting freewheel. */
			pwm_sr_width = 0;
		else
			pwm_sr_width = 512;			/* TIMER_TOP + 1 */
	}

	/* Record the operating extremes. */
	{
		extern int motor_pwm_min, motor_pwm_max;
		if (pwm_width < motor_pwm_min)
			motor_pwm_min = pwm_width;
		if (pwm_width > motor_pwm_max)
			motor_pwm_max = pwm_width;
	}
	/* Set the hardware pulse width. */
#if !defined(set_pwm_width)
#warning "Macro set_pwm_width() is not defined."
#endif
	set_pwm_width(pwm_width, pwm_sr_width);

	/* Calculate the average pulse width over time.
	 * The max value is 511, so we can multiply it by up to 127 times
	 */
#if 1
	{
		static uint16_t pwm_avg_16 = 0;
		uint8_t filter_shift = 7 - (config.pwm_filter & 3);

#if defined(USE_32BIT_PWM_AVG)
		luv1 = (uint32_t)pwm_width << 7;
		pwm_avg_32 = ((pwm_avg_32 << shift) - pwm_avg_32 + luv1) >> filter_shift;
#endif
		pwm_avg_16 += ((pwm_width<<7) - pwm_avg_16) >> filter_shift;
	}
#else
	luv1 = (uint32_t)pwm_width << 16;
	switch ((unsigned char)config.pwm_filter) {
	case 0:
		pwm_avg_32 = ((pwm_avg_32 * 127) + luv1) >> 7;
		break;
	case 1:
		pwm_avg_32 = ((pwm_avg_32 * 63) + luv1) >> 6;
		break;
	case 2:
		pwm_avg_32 = ((pwm_avg_32 * 31) + luv1) >> 5;
		break;
	case 3:
		pwm_avg_32 = ((pwm_avg_32 * 15) + luv1) >> 4;
		break;
	default:
		pwm_avg_32 = 0;
	}
	pwm_avg = pwm_avg_32 >> 16;
#endif

	/* Rotate among four different calcluations, each at a 1KHz rate. */
	calc_phase++;
	switch (calc_phase & 0x03) {
	case 0x00: {
		/* Throttle normalization: calculate throttle_ref.
		 * This typically converts from the A/D 0-5V input
		 * (really about 1V-4V) to [0..511].  We check extreme values,
		 * which could indicate a mechanical or wiring failure.
		 * This could be done only when there is a new throttle A-D
		 * conversion, but the code path here is short.
		 * We also check for a CAN bus throttle message, which is 
		 */
		int throttle_base;

		/* Fade old faults. */
		if (throttle.fault_counts > 0)
			throttle.fault_counts--;

		/* A remote throttle message overrides the locally connected
		 * throttle.
		 */
		if (throttle.remote_valid) {
			if (throttle.remote_time - clock_1msec > THROTTLE_REMOTE_TIMEOUT) {
				throttle.remote_valid = 0;
				/* Set the high pedal lockout so that switching back to local
				 * throttle control requires releasing the pedal.
				 */
				fault_bits |= HPL_FAULT;
				throttle_ref = 0;
			} else {
				throttle_ref = throttle.remote << 1;
			}
			break;
		}
		/* Record the operating extremes. */
		{
			extern int raw_throttle_max, raw_throttle_min;
			if (raw_throttle > raw_throttle_max)
				raw_throttle_max = raw_throttle;
			if (raw_throttle < raw_throttle_min)
				raw_throttle_min = raw_throttle;
		}
		/* Normalize throttle and scale to [0..511] with minimal ops. */
		throttle_base = raw_throttle - throttle.closed;
		if (throttle_base < 0 || throttle_base > throttle.range) {
			throttle_ref = 0;
			if (raw_throttle < throttle.low_fail ||
				raw_throttle > throttle.high_fail) {
				throttle.fault_counts += 2; 	/* Note fault "fade" above. */
				if (throttle.fault_counts >= THROTTLE_FAULT_COUNTS) {
					throttle.fault_counts = THROTTLE_FAULT_COUNTS;
					/* We should record OBD-DTC info here. */
					fault_bits |= THROTTLE_FAULT;
				}
			} else if (throttle_base > throttle.range)
				throttle_ref = 511;				/* Just floored, not broken. */
		} else						/* Linear range. */
			throttle_ref = (throttle_base * throttle.gain) >> 6;
		break;
	}
	case 0x01: {
		/* Throttle mapping: calculation of current_ref from throttle_ref.
		 * Even with no new throttle setting this needs to be run at 1KHz
		 * for a smooth ramp-up. */
		int new_current;
		int current_delta;

		new_current = throttle_ref*config.throttle_pos_gain
			- pwm_avg*config.throttle_pwm_gain;
		if (new_current < 0)
			new_current = 0;
		else
			new_current >>= 3;		/* Normalize after gain multiply.  */

#if defined(RPM_REDLINE)
		/* Check our RPM limit. */
		if (tach_redline_qrpm && tach_get_QRPM() > tach_redline_qrpm)
			new_current = 0;
#endif

		/* Ramp rate control and upper limit check. */
		/* current_delta = MIN(max_current_ref, raw_throttle) - current_ref; */

		if (new_current < max_current_ref)
			current_delta = new_current - current_ref;
		else
			current_delta = max_current_ref - current_ref;

		/* Bound the change in current by current_ramp_rate. */
		if (abs(current_delta) > config.current_ramp_rate) {
			if (current_delta > 0)
				current_ref += config.current_ramp_rate;
			else
				current_ref -= config.current_ramp_rate;
		} else
			current_ref += current_delta;

		/* Limit battery current based on a EEPROM-configured or dynamic
		 * value.  If we don't have a separate sensor, estimate battery
		 * amps as motor amps time PWM percent.
		 */
		if (config.battery_amps_limit > 0) {
			/* Filter to eliminate transient current peaks. */
			if (pwm_avg > 0) {
				unsigned uv2;
				luv1 = bat_amp_lim_510 / (uint32_t)pwm_avg;
				if (luv1 < MAX_CURRENT_REF) {
					uv2 = luv1;
					if (current_ref > uv2)
						current_ref = uv2;
				}
			}
		}
		break;
	}
	case 0x02: {
		/* Estimate battery current and accumulate amp-hours used.
		 *  battery_amps = motor_amps * pulse%  = current_fb * PWM/512;
		 * We round up, knowing that this a optimistic estimate. */
		uint32_t battery_amps_l;

		battery_amps_l = (uint32_t)current_fb * (uint32_t)pwm_width;
		battery_amps = (battery_amps_l + 0x0100) >> 9;

		/* Add to amp-hour accumulator, with a limit check. */
		battery_ah += battery_amps;
		if (battery_ah & 0x80000000)
			battery_ah = 0x7FFFFFFF;
		break;
	}
	case 0x03:
		/* 1KHz check for motor overspeed if enabled. */
		if (config.motor_os_th > 0) {
			motor_overspeed_fault_count++;
			if (fault_bits & MOTOR_OS_FAULT) {
				/* Reset fault when it gets old. */
				if (--motor_os_fault_timer <= 0)
					fault_bits &= ~MOTOR_OS_FAULT;
			} else {
				/* Check for a new overspeed fault. */
				if (pwm_width > config.pwm_deadzone)
					luv1 = (uint32_t)(pwm_width - config.pwm_deadzone) << 16;
				else
					luv1 = 0;

				/*
				 * original logic in v1.11
				 * if current feedback > motor_speed_calc_amps,
				 *  then rpm = k * V / current_feedback
				 *  else rpm = 0
				 */
#if 0
				if (current_fb > config.motor_sc_amps)
					luv1 = luv1 / (uint32_t)current_fb;
				else luv1 = 0;
#endif

				// logic changed slightly in v1.11b
				// if current feedback > motor_speed_calc_amps, then rpm = k * V / current_feedback
				// else rpm = k * V / motor_speed_calc_amps
				if (current_fb > config.motor_sc_amps) luv1 = luv1 / (uint32_t)current_fb;
				else luv1 = luv1 / (uint32_t)(config.motor_sc_amps + 1);

				if (luv1 > motor_overspeed_threshold) {
					if (motor_os_count < (unsigned char)config.motor_os_dt) motor_os_count++;
					else {
						// we have motor overspeed
						fault_bits |= MOTOR_OS_FAULT;
						motor_os_count = 0;
						motor_os_fault_timer = config.motor_os_ft;
					}
				}
				else motor_os_count = 0;
			}
		}
		break;
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
