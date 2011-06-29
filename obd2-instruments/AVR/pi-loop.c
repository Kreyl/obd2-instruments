/* pi-loop.c: Proportional-integral loop for VVVVroom instruments. */
/*
 * The optional Proportional-Integral loop code.
 *
 * This outline provides a place to add a control loop to the instrument
 * firmware, typically a proportional-integral loop.
 *
 * A P-I control loop is a feedback controller which drives the output
 * with a weighted sum of the error (difference between the output and
 * desired set-point) and the integral of that value.
 *
 * Using the integral term causes the steady-state error to reduce to zero,
 * which is not usually the case for proportional-only control.
 *
 * This loop runs at 4KHz, just after a new sample of the primary control
 * A/D channel.
 */
static const char versionA[] =
"$Id: pi-loop.c 144 2011-05-31 14:46:55Z becker $ Copyright Donald Becker\n";

#include <avr/io.h>
#include <avr/pgmspace.h>
#include "vvvvroom.h"
#include "command.h"

struct pi_state {
	int32_t K1;					/* Scaled Kp (proportional constant) */
	int32_t K2;					/* Scaled Ki (integral feedback constant) */
	int16_t error_old;
	uint32_t pwm;
};
static struct pi_state pi;

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
	/* Informative message about the P-I loop. */
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
int control_fb;			  /* Controlled signal feedback e.g. actual current. */
int current_ref;		  /* Reference (desired) result e.g. target current. */

void pi_loop(void)
{
	int16_t current_error;

	/* The control function typically operates on raw_adc[] values. */
	control_fb = (raw_adc[0] * 19) >> 3;

	current_error = current_ref - control_fb;
	pi.pwm += (pi.K1 * current_error) + (pi.K2 * pi.error_old);
	pi.error_old = current_error;


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
