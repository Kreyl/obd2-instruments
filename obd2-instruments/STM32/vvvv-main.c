/* vvvv-main.c: STM32 Firmware for the VVVVroom motor controller. */
/*
	Written 2010-2011 by Donald Becker and William Carlson.

	The VVVVroom motor controller firmware rewritten for the STM32 processor.
	This file contains the main control, both main() and the primary
	motor control feedback loop.

	The hardware configuration is as follows
	Timer1  Motor control PWM timer.  Channel 1 is the drive output,
	with Channel 1N a freewheel/SR output.  Channel 2 and 3 are reserved

*/
static const char versionA[] =
"$Id: vvvv-main.c $\n";

#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#if defined(STM32)
#include "armduino.h"
#include "vvvvroom.h"
#include "can.h"

/* Definitions for the Discovery board. */
#define LED_BLUE	(1<<8) // pin 8
#define LED_GREEN	(1<<9) // pin 9
#define cli()
#define sei()

#define show_banner() \
	serprintf(PSTR("VVVVRoom controller firmware vSTM0.01\r\n"));

#endif

/* We match the EEPROM configuration layout of the Cougar firmware where
 * possible.  This makes it possible to use this code as an upgrade path.
 * The additional configuration info we need is appended.
 */

/* The default configuration to use if the EEPROM is empty or
 * corrupted.
 */
config_type PROGMEM default_config = {
 magic: 0x12ab,					 /* Magic ID, matches Cougar config. */
 Kp:	2,						 /* PI loop Proportional gain */
 Ki:	160,					 /* PI loop Integral gain */
 /* Throttle thresholds, in A/D converter units.
  * Note that Cougar used 5-0V e.g. high = closed throttle, low = pedal down.
  * We use the more common 0V-5V setup.
  */
 throttle_min_raw_counts:	1930,
 throttle_max_raw_counts:	4095,
 throttle_fault_raw_counts:	100,
 /* The throttle pedal has a fixed gain setting, not proportional to
  * configured range. */
 throttle_pos_gain:	2,			/* Gain/8 e.g. 8 is *1.0 */
 throttle_pwm_gain:	0,			/* Gain/8. Adjust accel vs. decel gain */
 current_ramp_rate:	6,
 rtd_period:	0,				 /* RTD (real time data) report period msec */
 pwm_filter:	0,				 // PWM filter
 motor_os_th:	0,				 // motor overspeed threshold
 motor_os_ft:	1000,			 // motor overspeed fault time
 motor_os_dt:	10,				 // motor overspeed detect time
 pwm_deadzone:	5,				 /* 1.2 usec minimum on-time. */
 battery_amps_limit:	0,
 precharge_time:	0,
 motor_sc_amps:	0,
 /* Non-Cougar settings.  Up to five 16 bit parameters. */
 motor_amps_limit: 60,
 spares: {0,},
 crc: 0
};

/* The most recently read sensor values, and when they were read. */
volatile uint16_t raw_adc[NUM_ADC_CHANNELS];
volatile unsigned raw_adc_sample_time[NUM_ADC_CHANNELS];

/* Statistics and performance monitoring. */
struct perfstats {
	int timer1_entry;			/* Timer1 value at interrupt Entry/exit */
	int timer1_exit;
} stats;

struct fault_info fault;

/* Cougar variables.
 * A timer for the Over-Current lockout, and a flag that we are
 * already in the PI loop.*/
unsigned char oc_cycles_off_counter = 0;
unsigned char in_pi_loop = 0;

#if defined(MEGA1280)
#define LED_TURNON PORTB |= PB_LED;	/* Force the Mega onboard LED on. */
#define TOGGLE_LED PORTB ^= PB_LED;	/* Toggle the Mega LED */
#define close_contactor() PORTD &= ~PD_CONTACTOR;
#elif defined(COUGAR)
#define LED_TURNON PORTD |= PD_LED;	/* Force the Cougar "idle" LED on. */
#define TOGGLE_LED PORTD ^= PD_LED;	/* Toggle the Cougar LED */
#define close_contactor() PORTD &= ~PD_CONTACTOR;

#elif defined(STM32)
#define close_contactor() GPIOC_BRR = PD_CONTACTOR;
#define LED_TURNON GPIOC_BSRR = LED_BLUE | LED_GREEN;
#if defined(QAR)
/* The QAR board has the LED on PB0 */
#define TOGGLE_LED GPIOB_ODR = PORTB ^ 1;
#else
/* The VLDiscovery */
#define TOGGLE_LED GPIOC_ODR = PORTC ^ (LED_BLUE | LED_GREEN);
#endif

/* A sleazy wait-loop delay.  Use only for debugging before timers work. */
#if 0
void delay(void)
{
	register unsigned int i;
	for (i = 0; i < 2000000; ++i)
		__asm__ __volatile__ ("nop\n\t":::"memory");
	return;
}
#endif

#else
#error Uknown hardware platform
#endif

unsigned baseline_m_current;	 /* Current sensor zero current baseline. */
unsigned lowfault_m_current;	 /* A/D reading that indicates a fault.  */
uint16_t m_current_peak = 0;

/* Use direct PWM for if the current sensor breaks -- dangerous!
 * If this is non-zero, it is the limit when PWM is proportional to the
 * throttle. */
uint16_t use_direct_pwm = 0;

unsigned battery_amps = 0;				// calculated battery current
uint32_t battery_ah = 0;			// calculated battery AH used by controller

uint32_t bat_amp_lim_510 = 0;		// battery amps limit multiplied by 510 (max PWM)

uint32_t motor_overspeed_threshold = 0;	// motor overspeed threshold
unsigned motor_os_fault_timer = 0;				// motor overspeed fault timer (milliseconds)
unsigned char motor_os_count = 0;				// motor overspeed debounce timer (milliseconds)
uint32_t motor_overspeed_fault_count = 0;

unsigned precharge_timer = 0;			// precharge timer in 0.1 second units
int16_t overheat;			/* A positive value indicates overheat */

unsigned throttle_fault_counts = 0;
#if 0
volatile unsigned char fault_bits = HPL_FAULT;
#endif

uint32_t idle_loopcount;			// how many loops we do while micro is not executing PI

unsigned tm_show_data;					// timer for realtime data display
unsigned tm_can_pkt;					/* Timer for CAN status heartbeat. */
unsigned can_beat_period = 976;			/* Period between CAN heartbeats. */

config_type config;
realtime_data_type rt_data;

inline unsigned get_time(void)
{
	return clock_1msec;				/* Must be atomic */
}

inline unsigned diff_time(unsigned before)
{
	return clock_1msec - before;
}

/* Records the number of loops we can do in the specified period.
 * A calibration cycle is done while idle, and later compared with
 * the loop cound while operating.
 */
uint32_t wait_time(unsigned howlong)
{
	unsigned begin;
	unsigned long loopcount;

	loopcount = 0;
	begin = get_time();
	while (diff_time(begin) < howlong)
		loopcount++;
	return loopcount;
}

/* Primary PWM timer overflow interrupt.
 * Most of the control loop and real time monitoring happen in this
 * function.
 *
 * This interrupt is center aligned with the PWM drive pulse - a good time to
 * sample current sensors.  Although ideally we would sample the
 * current on the center of both on and off pulses to average in the
 * freewheel current.
 *
 * Unlike the AVR, the STM32 has a high speed A/D converter, fast enough
 * to sample both current sensors every cycle while still monitoring the
 * other sensors.
 */
ISR(TIM1_UP_TIM16)
{
	static unsigned char counter_16k = 0;

	/* The A/D converter channel of the most recent conversion start. */
	static unsigned char adc_channel = 0;

	stats.timer1_entry = TIM1_CNT;
	/* Clear the interrupt source. */
	TIM1_SR = 0;

	counter_16k++;
	if (counter_16k & 0x01) {

		/* The "odd" cycle A/D conversion steps through the less time
		 * sensitive sensors such the throttle, voltages or temperatures.
		 * We assume that the conversion is complete and just read the
		 * result.  If the PWM frequency can vary, it would be better to
		 * use read_adc_voltage(), or at least check for completion.
		 */
		raw_adc[adc_channel] = read_adc_voltage();
		raw_adc_sample_time[adc_channel] = counter_16k;
		/* Immediately start an A/D conversion on the output current. */
		start_adc_conversion(ADC_CHANNEL_M_CURRENT);
	} else {
		/* Even phase is current/amps sensor reading complete. */
		raw_adc[ADC_CHANNEL_M_CURRENT] = read_adc_voltage();
		/* Read all channels as they may be queried over CAN. */
		if (++adc_channel >= NUM_ADC_CHANNELS)
			adc_channel = 0;
		start_adc_conversion(adc_channel);

		/* The PI loop should never take so long that we re-enter, but we
		 * copy the check in the Cougar.
		 */
		if (!in_pi_loop) {
			in_pi_loop = 1;
			sei();
			pi_loop(raw_adc[ADC_CHANNEL_M_CURRENT],
					raw_adc[ADC_CHANNEL_THROTTLE]);
			cli();
			in_pi_loop = 0;
		} else			/* ToDo: set a DTC */
			fault.pi_overwork++;
	}
	stats.timer1_exit = TIM1_CNT;
	return;
}

/* Copy Real-Time Data into the active structure.
 * The Cougar code did this one at a time, disabling interrupts with
 * each copy.  That is pointlessly slow.
 */
void fetch_rt_data(void)
{
	cli();
	rt_data.throttle_ref = (volatile int)throttle_ref;
	rt_data.current_ref = (volatile int)current_ref;
	rt_data.current_fb = (volatile int)current_fb;
	rt_data.raw_hs_temp = (volatile unsigned)raw_adc[ADC_CHANNEL_TEMP];
	rt_data.raw_throttle = (volatile unsigned)raw_adc[ADC_CHANNEL_THROTTLE];
	rt_data.battery_amps = (volatile unsigned)battery_amps;
	rt_data.battery_ah = (volatile unsigned long)battery_ah;
	sei();
#if ! defined(STM32)
	asm("nop"); asm("nop"); asm("nop"); asm("nop");
#endif
}

/*
 * Thermal cutback.
 * This function modifies max_current_ref to communicate the limit.
 * If the heatsink temperature exceeds the threshold, we use the ADC value
 * directly as the proportion to limit the throttle.
 * This works out well with the LM335, 10mV/C and 5mV/count results in
 * dropping the current to 0 over a 50C range.
 * We don't need a smooth curve here.  If the controller is close to full
 * overheat, we want to shut down rather than inefficiently getting 10% power.
 */
void thermal_cutback(void)
{
	uint16_t new_current_limit;
	overheat = raw_adc[ADC_CHANNEL_TEMP] - THERMAL_CUTBACK_START;

	if (overheat > 100)			/* About +50C over hot threshold. */
		new_current_limit = 0;
	else if (overheat > 0)		/* Drop 4 units per count,  1/64 per C */
		new_current_limit = MAX_CURRENT_REF - (overheat<<2);
	else
		new_current_limit = MAX_CURRENT_REF;
	if (new_current_limit != max_current_ref) {
		cli(); max_current_ref = new_current_limit; sei();
	}
	return;
}

int main(void)
{
	unsigned tm_100, tm_last_command;

	/* Clock setup is now done in setup_io_ports(). */

	/* We may re-start from a watchdog reset, so we prioritize setting the
	 * outputs to a safe state and enabling status reporting.
	 */
	fault.bits = HPL_FAULT;
	in_pi_loop = 0;
	setup_io_ports();
	setup_uart();

	/* We optionally verify intact firmware before frobbing the traction
	 * circuit.  If the firmware is corrupt, this will power off. */
	if ( ! check_firmware_integrity()) {
		/* Do something useful, like hang. */
	}

#if 0
	/* We have to turn on interrupts to communicate. */
	asm volatile ("cpsie i");
	/* Set the SYSTICK priority */
	/* SYSTEM_HANDLER_12_15_PRIORITY_REGISTER |= (SYSTICK_PRIORITY << 24); */
#endif

	show_banner();
	setup_clock();				/* ARM core SysTick timer. */
	adc_setup();
	CAN_init();

	/* Persistent config from EEPROM. This may eventually contain
	 * options for configuring the I/O ports, but for now we prefer that
	 * the I/O ports are set up well enough to report errors.
	 */
	read_config();
#if 0
	show_config(0xffff);
#endif

	/* Read out the pre-start voltages.
	 * We can only do this before the timers start. */
	show_adc_voltages();

	/* Start the pre-charge process.
	 * With a zero period, there is no precharge capability.
	 * With no precharge control, the time is a fixed wait period.
	 * With capacitor voltage monitoring only, the time is a maximum period.
	 * With active precharge control, the time is an error limit.
	 */
	if (config.precharge_time > 0) {
		fault.bits |= PRECHARGE_WAIT;
		precharge_timer = config.precharge_time + 5;
	}

	/* Initialize the Proportional-Integral loop from the config structure. */
	pi_init();
	/* Wait 100mS, recording the number of loops.  We expect about 35960. */
	idle_loopcount = wait_time(100);
	serprintf(PSTR("Loops for 100mSec %d\r\n"), idle_loopcount);

	/* Enable watchdog.  We must do this before starting any power-control
	 * process. */
	watchdog_enable();

	/* Establish baseline quiescent voltages. */
	baseline_m_current = adc_baseline(ADC_CHANNEL_M_CURRENT);
	/* Threshold for a low-read fault is -0.5V down or 1024/10 */
	lowfault_m_current = baseline_m_current - 102;

	/* Set an error flag if any are out of range. */
	if ((baseline_m_current > (2048 + 200))
		|| (baseline_m_current < (2048 - 200))) {
		fault.bits |= VREF_FAULT;
		serprintf(PSTR("Baseline current reading %4d is out of range.\r\n"),
					   baseline_m_current);
		serprintf(PSTR("Using raw throttle position for PWM!\r\n"));
		use_direct_pwm = PROPORTIONAL_PWM_MAX;
	}

	setup_timers();
	setup_tach();
	wdt_reset();
	CAN_start();

	/* Init some time variables. */
	tm_can_pkt = tm_show_data = tm_100 = get_time();
	tm_last_command = tm_show_data - 10000;
	serprintf(PSTR("> "));

	/* Our operating loop is mostly non-time-critical code.
	 * The interesting stuff happens in the interrupt handlers.
	 */
	while (1) {
		int x;
		wdt_reset();

		/* See if we have any communication waiting. */
		x = uart_getchar();
		if (x >= 0) {
			do_serial_port_char((unsigned char)x);
			tm_last_command = get_time();
		}

		CAN_poll();			/* Poll the CAN bus controller. */

		/* Get a fresh copy of our sensor values. */
		fetch_rt_data();
		/* Update the thermal cutback based on the new values. */
		thermal_cutback();

		/* The cougar controller had a mode to periodically spew a line
		 * with operating parameters.  This would be better done with CAN,
		 * but there is already a display app for this serial data format.
		 * Suppress output for about 10 seconds if there was recent serial
		 * input.
		 */
		if (config.rtd_period &&
			diff_time(tm_show_data) >= config.rtd_period &&
			diff_time(tm_last_command) > RTD_SILENCE_ON_INPUT) {
			show_cougar_rt_data();
			tm_show_data += config.rtd_period;	/* Time for next report. */
			if (tm_show_data - get_time() > 11000)
				tm_show_data = get_time();
			/* Drag-along to avoid false trigger. */
			tm_last_command = get_time() - (10000+1);
		}
		/* Do the 0.1Hz processing. */
		if (diff_time(tm_100) >= 100) {
			tm_100 += 100;		/* Alarm time for next 100msec update. */
			/* This logic is from the Cougar. It should be a more general
			 * event state machine. */
			// if fault, toggle LED, else light LED
			if (fault.bits) {
				TOGGLE_LED;
				if (fault.bits & HPL_FAULT) {
					/* High pedal lockout fault set - reset it if
					 * current ref is below threshold */
					if (rt_data.throttle_ref <= HPL_THROTTLE_THRESHOLD) {
						cli(); fault.bits &= ~HPL_FAULT; sei();
					}
				}
				if (fault.bits & PRECHARGE_WAIT) {
					if (precharge_timer == 0) {
						/* Remove precharge wait condition to enable
						 * power to motor */
						cli(); fault.bits &= ~PRECHARGE_WAIT; sei();
					} else {
						/* With half a second left in precharge, close contactor */
						if (precharge_timer <= 5)
							close_contactor();
						precharge_timer--;
					}
				}
			} else {
				LED_TURNON;
			}
			/* Send out a CAN heartbeat. */
			if (can_beat_period &&
				diff_time(tm_can_pkt) >= can_beat_period) {
				CAN_heartbeat();
				tm_can_pkt += can_beat_period;	/* Time for next report. */
			}
		}
	}

	return 0;					/* No exit occurs. */
}

/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
