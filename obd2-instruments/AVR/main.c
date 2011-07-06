/* vvvvroom.c: Firmware for the VVVVroom motor controller. */
/*
	Written 2010-2011 by Donald Becker and William Carlson.

	The structure and some of this code comes from the Cougar firmware.
*/
static const char versionA[] =
"$Id: vvvvroom.c 157 2011-03-31 00:13:59Z becker $ Copyright Donald Becker\n";

#define show_banner() \
	serprintf(PSTR("VVVVRoom instrument controller firmware AVR-0.01\r\n"));

#if defined(__AVR_ATmega168__)
#define MEGA168
#define COUGAR
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
#include <util/crc16.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vvvvroom.h"
#include "can.h"
#include "command.h"

#if ! defined(PWM1_OUT_SETTING)
/* TCCR1A Set the output polarity based on invert/non-inverting driver.
 * OCR1A is the low-side gate driver
 * OCR1B is the high side gate if sync rectification/SR freewheel is used.
 * OCR1C is the LED output, used for debugging.
 * Set to 2 to clear (0V output) on up count, set (5V) on down count.
 * Set to 3 for a low-turns-on inverting drive (5V on up match, 0V down).
 * Note that the bootloader drives the OCR1A/B pins high just after reset
 * so using an active-low gate driver is much preferred!
 */
#define PWM_OUT_SETTING	((3 << COM1A0) | (2 << COM1B0) | (2 << COM1C0))
#endif

#if defined(IOM8)
#define TIMSK1 TIMSK
#define ICIE1  TICIE1
#endif

/* If AUTOCRC is defined, use automatically generated CRC file */
#ifdef AUTOCRC
#include "autocrc.h"
#endif

/* We match the EEPROM configuration layout of the Cougar firmware where
 * possible.  This makes it possible to use this code as an upgrade path.
 * The additional configuration info we need is appended.
 */

/* The default configuration to use if the EEPROM is empty or
 * corrupted.
 */
config_type default_config PROGMEM = {
 magic: 0x12ab,					 /* Magic ID, matches Cougar config. */
 Kp:	2,						 /* PI loop Proportional gain */
 Ki:	160,					 /* PI loop Integral gain */
 /* Throttle thresholds, in A/D converter units.
  * Note that Cougar used 5-0V e.g. high = closed throttle, low = pedal down.
  * We use the more common 0V-5V setup.
  */
 throttle_min_raw_counts:	225,
 throttle_max_raw_counts:	800,
 throttle_fault_raw_counts:	100,
 /* The throttle pedal has a fixed gain setting, not proportional to
  * configured range. */
 throttle_pos_gain:	5,			/* Gain/8 e.g. 8 is *1.0 */
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
 motor_amps_limit: 130,
 spares: {0,},
 crc: 0
};

/* The most recently read sensor values, and when they were read. */
volatile unsigned raw_adc[NUM_ADC_CHANNELS];
volatile unsigned raw_adc_sample_time[NUM_ADC_CHANNELS];

/* The recent value read from the PortD pins. */
int portd_value;

/* Statistics and performance monitoring. */
int stats_timer1_entry;			/* Timer1 value at interrupt Entry/exit */
int stats_timer1_exit;

/* Cougar variables.
 * A timer for the Over-Current lockout, and a flag that we are
 * already in the PI loop.*/
unsigned char oc_cycles_off_counter = 0;
unsigned char in_pi_loop = 0;

#if defined(MEGA1280)
#define LED_TURNON PORTB |= PB_LED;	/* Force the Mega onboard LED on. */
#define TOGGLE_LED PORTB ^= PB_LED;	/* Toggle the Mega LED */
#elif defined(COUGAR)
#define LED_TURNON PORTD |= PD_LED;	/* Force the Cougar "idle" LED on. */
#define TOGGLE_LED PORTD ^= PD_LED;	/* Toggle the Cougar LED */
#else
#error Uknown hardware platform
#endif

/* We keep the time in three variables.
 * An 8KHz counter, used only within the Timer 1 Overflow interrupt handler.
 * A 1KHz 16 bit counter, used for most timing.
 * An upper 16 bits counter incrementing on the 1KHz overflow, available for
 * time stamps.  About one tick a minute (67.1 seconds).
 */
static unsigned char counter_8k = 0;
volatile unsigned clock_1msec = 0;		/* 1KHz (976Hz to be exact) counter */
unsigned time_high = 0;					/* Extension to clock_1msec. */

unsigned baseline_m_current;	 /* Current sensor zero current baseline. */
unsigned lowfault_m_current;	 /* A/D reading that indicates a fault.  */
uint16_t m_current_peak = 0;

/* Use direct PWM for if the current sensor breaks -- dangerous!
 * If this is non-zero, it is the limit when PWM is proportional to the
 * throttle. */
uint16_t use_direct_pwm = 0;
volatile uint16_t pwm_width = 0x123;

unsigned max_current_ref = 0;			// max_current_ref in variable so controlled by temperature
int throttle_ref = 0;					// reference (desired) throttle
int current_ref = 0;					// reference (desired) current
int current_fb = 0;						// current feedback (actual current)

unsigned battery_amps = 0;				// calculated battery current
unsigned long battery_ah = 0;			// calculated battery AH used by controller

unsigned long bat_amp_lim_510 = 0;		// battery amps limit multiplied by 510 (max PWM)

unsigned long motor_overspeed_threshold = 0;	// motor overspeed threshold
unsigned motor_os_fault_timer = 0;				// motor overspeed fault timer (milliseconds)
unsigned char motor_os_count = 0;				// motor overspeed debounce timer (milliseconds)
unsigned long motor_overspeed_fault_count = 0;

unsigned precharge_timer = 0;			// precharge timer in 0.1 second units

struct fault_info fault = {
 bits: INITIAL_FAULTS,
};

unsigned long idle_loopcount;			// how many loops we do while micro is not executing PI

unsigned tm_show_data;					// timer for realtime data display
unsigned tm_can_pkt;					/* Timer for CAN status heartbeat. */
unsigned can_beat_period = 976;			/* Period between CAN heartbeats. */

config_type config;
realtime_data_type rt_data;

const unsigned int firmware_crc = 0x4242;
#ifdef crc_address
/* Check that the program matches the original.
 * This requires that the 16 bit CRC immediately follow the program in flash.
 * PROGSTART is usually 0x0000, but may be higher if the interrupt vectors is
 * in program space rather than boot space.
 * This uses the 0x8408 polynomial, the same as used in PPP.
 */
void check_firmware_integrity(void)
{
	unsigned n, firmware_crc;

	firmware_crc = 0xffff;
	for (n = PROGSTART; n < crc_address; n++) {
		firmware_crc = _crc_ccitt_update (firmware_crc, pgm_read_byte(n));
	}

	/* Spew forever, just slow enough to see the LEDs blink. */
	while (pgm_read_word(PROGSTART + crc_address) != firm_crc) {
		char i;
		for (i = 127; i > 0; i--) {
			serprintf(PSTR("Firmware corrupt, CRC does not match\n"));
			show_banner();
		}
		TOGGLE_LED;
	}
	return;
}
#else
void check_firmware_integrity(void)
{
	serprintf(PSTR("Firmware integrity check not available %4x\n"),
			  firmware_crc);
}
#endif

/* Calculate the CRC on a region.
 * This is used to check the EEPROM configuration integrity.
 */
unsigned int calc_block_crc(unsigned nbytes, unsigned char *buf)
{
	unsigned n, crc;

	crc = 0xffff;
	for (n = 0; n < nbytes; n++) {
		crc = _crc_ccitt_update (crc, *(buf + n));
	}
	return crc;
}

#ifdef MEGA168
void watchdog_disable(void)
{
	asm ("cli");
	asm ("wdr");
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0x00;
	asm ("sei");
}
void watchdog_enable(void)
{
	asm ("cli");
	asm ("wdr");
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDE) | (1 << WDP2);
	asm ("sei");
}
#else
void watchdog_disable(void)
{
	wdt_reset();
	wdt_disable();
}
void watchdog_enable(void)
{
	wdt_reset();
	wdt_enable(WDTO_250MS);
}
#endif

#if defined(OC_CLEAR_ENABLED)
/* The Cougar board has a hardware overcurrent latch that blocks the PWM
 * motor pulses when triggered by an analog comparator.  The software
 * must reset the latch by driving a GPIO pin low for the latch reset
 * time.  This should be under 60ns for the two 74HC00 gate delay, but
 * the Cougar firmware waits 4 instruction cycles.  There is not to much
 * gain in 'fixing' this without having their hardware to verify.
 */
inline void clear_oc(void)
{
	PORTB &= ~PB_OC_CLEAR;
	asm("nop"); asm("nop");
	asm("nop"); asm("nop");
	PORTB |= PB_OC_CLEAR;
	return;
}
#endif

/* Time functions.
 * The time is based on the 1KHz (really 16MHz/2^14=976Hz) ticks.
 * With an unsigned 16 bit value we can span about a minute before
 * rolling over.
 */
inline unsigned get_time(void)
{
	unsigned t;

	cli(); t = clock_1msec; sei();
	return t;
}

inline unsigned diff_time(unsigned before)
{
	unsigned now;
	now = get_time();
	return now - before;
}

/* Records the number of loops we can do in the specified period.
 * A calibration cycle is done while idle, and later compared with
 * the loop cound while operating.
 */
unsigned long wait_time(unsigned howlong)
{
	unsigned begin;
	unsigned long loopcount;

	loopcount = 0;
	begin = get_time();
	while (diff_time(begin) < howlong)
		loopcount++;
	return loopcount;
}

/* The motor PWM control timer overflow interrupt
 * Most of the control loop and real time monitoring happens in this
 * function.
 *
 * This interrupt center aligned with the PWM drive output - a good time to
 * sample the current sensor.  Although ideally we would sample the
 * current on the center of both on and off pulses to average in the
 * freewheel current.
 *
 * We alternate between sampling the motor output current and measuring
 * other sensors.  This gives us a 4KHz sample to check for overcurrent,
 * with a minimum 250Hz rate (with all 16 ADC channels monitored) for
 * less critical sensors.
 */
ISR(TIMER3_OVF_vect)
{
	/* The A/D converter channel of the most recent conversion start. */
	static unsigned char adc_channel = 0;

	/* We are limited by the ADC conversion rate and thus run the sample
	 * processing at an 8KHz rate, even when the PWM is 16KHz.
	 */
	static unsigned char counter_16k = 0;

	counter_16k++;
	if (counter_16k & 0x01)
		return;

	counter_8k++;
	if (counter_8k & 0x01) {
		int fresh_portd_value;

		/* The "odd" cycle A/D conversion steps through the less time
		 * sensitive sensors such the throttle, voltages or temperatures.
		 * We assume that the conversion is complete and just read the
		 * result.  If the PWM frequency can vary, it would be better to
		 * use read_adc_voltage(), or at least check for completion.
		 */
		raw_adc[adc_channel] = ADC;
		raw_adc_sample_time[adc_channel] = counter_8k;
		/* Immediately start an A/D conversion on the output current. */
		start_adc_conversion(ADC_CHANNEL_M_CURRENT);
		if ((counter_8k & 0x06) == 0) {
			clock_1msec++;	/* 1 KHz counter for delays, etc. */
			if (clock_1msec == 0) { 		/* Manual overflow */
				time_high++;
			}
		}
		fresh_portd_value = PIND;

		if (fresh_portd_value != portd_value) {
			if ((fresh_portd_value ^ portd_value) & 0x01) { /* Tach tickled. */
				tach_tick(counter_8k, portd_value & 0x01);
			}
			portd_value = fresh_portd_value;
		}
#ifdef OC_CLEAR_ENABLED
		/* Cougar over-current trip handling.  Leave it tripped for
		 * a few cycles. */
		if (PINB & PINB_OC_STATE) {
			oc_cycles_off_counter++;
		}
		if (oc_cycles_off_counter >= NUM_OC_CYCLES_OFF) {
			// time to reset overcurrent trip circuit
			oc_cycles_off_counter = 0;
			clear_oc();
		}
#endif
	} else {
		/* Even phase is current/amps sensor reading complete. */
		raw_adc[ADC_CHANNEL_M_CURRENT] = ADC;
		/* Read all channels as they may be queried over CAN. */
		if (++adc_channel >= NUM_ADC_CHANNELS)
			adc_channel = 0;
		start_adc_conversion(adc_channel);

#if 0
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
			pi_overwork_faults++;
#endif
	}
	return;
}


#if EE_CONFIG_COPIES == 1
/* Read the configuration structure from the EEPROM.
 * If the magic number and CRC matches, use it directly.
 * Otherwise use the default config, without warning.
 */
void read_config(void)
{
	eeprom_read_block(&config, (void *)EE_CONFIG_ADDRESS, sizeof(config));
	if (config.magic == 0x12ab  &&
		calc_block_crc(sizeof(config) - sizeof(unsigned), (void *)&config)
		== config.crc) {
		/* Do some scaling. */
		motor_overspeed_threshold = (unsigned long)config.motor_os_th << 10;
		bat_amp_lim_510 = (unsigned long)config.battery_amps_limit * (unsigned long)510;
	} else {
		serprintf(PSTR("Invalid EEPROM contents, using default parameters\n"));
		memcpy_P(&config, &default_config, sizeof(config));
	}
	return;
}

void write_config(void)
{
	config.crc = calc_block_crc(sizeof(config) - sizeof(unsigned),
								(void *)&config);
	/* This might take a long time, so take the risk. */
	watchdog_disable();
	eeprom_write_block(&config, (void *)EE_CONFIG_ADDRESS, sizeof(config));
	watchdog_enable();
}
#else
void read_config(void)
{
	unsigned char lp, okmask;
	unsigned address;
	config_type cf;

	okmask = 0;
	address = EE_CONFIG_ADDRESS;
	for (lp = 0; lp < EE_CONFIG_COPIES; lp++) {
		okmask = okmask << 1;
		eeprom_read_block(&cf, (void *)address, sizeof(cf));
		address += sizeof(config_type);
		if (cf.magic == 0x12ab) {
			// magic OK
			if (calc_block_crc(sizeof(cf) - sizeof(unsigned), (unsigned char *)&cf) ==
			  cf.crc) {
			  	// CRC ok
				okmask = okmask | 0x01;
				memcpy(&config, &cf, sizeof(config));
				motor_overspeed_threshold = (unsigned long)config.motor_os_th << 10;
				bat_amp_lim_510 = (unsigned long)config.battery_amps_limit * (unsigned long)510;
			}
		}
	}
	if (okmask == 0) {
		// not even one good copy - copy default values and return
		serprintf(PSTR("Invalid EEPROM contents, using default parameters\n"));
		memcpy_P(&config, &default_config, sizeof(config));
		return;
	}
	// we have at least one good copy - repair any bad copies
	address = EE_CONFIG_ADDRESS;
	for (lp = 0; lp < EE_CONFIG_COPIES; lp++) {
		okmask = okmask << 1;
		if ((okmask & (1 << EE_CONFIG_COPIES)) == 0) {
			// copy is bad
			watchdog_disable();
			eeprom_write_block(&config, (void *)address, sizeof(config));
			watchdog_enable();
		}
		address += sizeof(config_type);
	}
}

void write_config(void)
{
	unsigned char lp;
	unsigned address;

	config.crc = calc_block_crc(sizeof(config) - sizeof(unsigned), (unsigned char *)&config);
	address = EE_CONFIG_ADDRESS;
	for (lp = 0; lp < EE_CONFIG_COPIES; lp++) {
		watchdog_disable();
		eeprom_write_block(&config, (void *)address, sizeof(config));
		watchdog_enable();
		address += sizeof(config_type);
	}
}
#endif

/* Configure the I/O port functions and directions.
 * This differs between hardware platforms, and the ATMega macros don't
 * make different ports easy to parameterize.
 */
void setup_io_ports(void)
{

	/* Port pin settings: function, direction, pull-up and initial values. */
#if defined(MEGA1280)
	/* Mega Port A and C are GPIO.
	 * PB0/1/2/3 is SPI port: SS/SCK/MOSI/MISO.
	 * PB5/6/7 are the Timer1 outputs OC1A/OC1B/OC1C (C is the LED)
	 * PE3/4/5 are the Timer3 outputs OC3A/OC3B/OC3C
	 * SPI pin direction is handled in the CAN and/or external ADC section.
	 * Set the other output pins on PB to high / weak pullups. */
	PORTB = 0xe0;				/* Set PWM outputs initially high. */
	DDRB = PB_LED | 0xe0;
	/* Enable UART1 Tx (PD3) as an output. */
	DDRD = 0x08;
	/* Enable the timer 3 OCxA/B/C and UART0 Tx (PE1) pins as outputs. */
	PORTE = 0x3A;
	DDRE = 0x3A;
	/* PD0-PD3, PE0-PE1 are used for communication.
	* The USART and ADC should be enabled by clearing
	* their bits in the PRR Power Reduction Register.
	*/
	PRR0 &= ~(PRADC | PRUSART0 | PRTIM1);
	PRR1 &= ~(PRTIM3);
	/* On the ATMega168 and later the digital input buffers must be
	 * disabled when the pins are used for the A/D converter.
	 * We enable all analog inputs, thus disable all digital buffers. pg296 */
	DIDR0 = 0xff;				/* ADC0-7 */
	DIDR2 = 0xff;				/* ADC8-15.  Note: not DIDR1 ! */
#else  /* Cougar */
	/* Port B on Cougar: PB1=PWM-out, PB2=OC-clear. */
	PORTB = 0xff & ~PB_PWM;		/* Make certain that the FETs are not driven */
	DDRB = PB_PWM | PB_OC_CLEAR;
	/* Port C
	 * For the Cougar, this controls the analog inputs.  Only three are used,
	 * mising the opportunity to measure parameters such as input current.
	 * The unused ports are set to weak pull-ups.
	 * On the Mega all Port C pins are GPIO.
	 */
	PORTC = ~PC_ANALOGS_USED;
	#ifdef MEGA168
	DIDR0 = (1 << ADC2D) | (1 << ADC1D) | (1 << ADC0D);
	#endif
	/* Port D on the Cougar
	 * PD0/PD1 RXD/TXD from RS232
	 * PD5 UnderVoltage Fault input
	 * PD6 Idle LED output (on when high)
	 * PD7 Contactor relay output
	 *  all get weak pull-ups, except for the LED output pin.
	 */
	PORTD = 0xff & ~PD_LED;
	DDRD = PD_CONTACTOR;
#endif

#if 1
	/* Timer channels.
	 * Temporarily set up timer channel 1, which normally drives the
	 * primary PWM output, to a simple up-counter.
	 * ToDo: This is used only for initialialization.  We should omit
	 * this temporary configuration and set up the final timer configuration.
	 */
	TCNT1 = 0;					/* Load 16 bit counter 1 */
	ICR1 = (long)F_CPU / 976;	/* Timer at 976 Hz */
	TCCR1A = 0;					/* no output action on match */
	/* Let counter 1 run at fosc, reset to 0 at ICR1 */
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);

	/* Enable input capture 1 interrupt  pg 168. */
	TIMSK1 = (1 << ICIE1);
#endif

	return;
}


/* Configure the Timer/PWM channels.
 * The PWM motor drive output is on the OC1A on Cougar
 * For now we simply configure timer 1 for PWM,
 * We configure the PWM to (pg 148)
 *  Mode 2  9 bits: 16000000 / (1 << 10) / 2 = 7,812Hz
 *  Mode 3 10 bits: 16000000 / (1 << 9) / 2 = 15,625Hz
 *  Mode 8/10 (variable on ICR1):  16000000/2 / ICR1, arbitrary frequency
 * The clock_1msec variable is incremented every 16 interrupt, at a rate of
 *  15625 / 16 = 976.5625Hz
 * Compare SIG_INPUT_CAPTURE1 running at 976Hz instead of 1KHz
 * The table of possible modes is page 148
 */
#if 1
/* Set the timer to Mode 10: flexible frequency, phase correct PWM. */
#define TIMER_MODE 10
#define TIMER_TOP  0X1FF
#elif defined(PWM8K)
/* Set the timer to Mode 3: phase correct PWM, 10 bits. */
#define TIMER_MODE 3
#define TIMER_TOP  0x3FF
#else
/* Set the timer to Mode 2: phase correct PWM, 9 bits. */
#define TIMER_MODE 2
#define TIMER_TOP  0x1FF
#endif

void setup_timers(void)
{
	/* Timer 0 is an 8 bit counter, left unused for now. */
	/* Reserve Timer 2 for future use as a real time clock. */
	/* Timer 1 or 3 (alternate) is the primary motor timer. */

	/* Set the motor control timer to a symmetrical PWM mode, 2,3,10:
	 * clear on match counting up, set on down match. */
	TIMSK3 = 0;					/* Disable interrupts during setup. */
	/* Stop the counter and configure the mode. */
	TCCR3B = ((TIMER_MODE & 0xC) << (WGM12 - 2));
	TCNT3 = 0;
	OCR3A = 0;				  /* Set the initial PWM duty cycle to 0. */
	OCR3B = TIMER_TOP;			/* SR gate drive off. */
	/* The timer mode and output polarities are set in defines. */
	TCCR3A = PWM_OUT_SETTING | ((TIMER_MODE & 0x3) << WGM10);
	ICR3 = TIMER_TOP;			/* Write after the mode is set. */
	/* Bypass pre-scaler to use 16MHz clock, re-set upper mode bits. */
	TCCR3B = (1 << CS10) | ((TIMER_MODE & 0xC) << (WGM12 - 2));
	OCR3A = 0;					/* Duty cycle to 0% again, just to be safe. */
	OCR3C = 100;
	OCR3B = TIMER_TOP;			/* SR gate drive off. */
	TIMSK3 = (1 << TOIE1);		/* Enable the overflow interrupt. */

	/* Timer 4 is used to clock the CAN controller.
	 * It needs a 50% duty cycle clock that's at least 10x (preferably 16x)
	 * the max bit rate, which may be 250K, 500K or 1000Kbps.
	 * To get a 8MHz out from a 16MHz sys clock we must use mode 4 or 12.
	 * We set to mode 4: clear timer on compare w/ OCR4A.
	 */
#define TIMER4_MODE 4
	TIMSK4 = 0;					/* Clear the interrupt flags. */
	TCCR4B = 0;					/* Stop the counter to load. */
	TCNT4 = 0xFFFF;				/* Loading with zero will skip a cycle! */
	OCR4A = 0;					/* Max count is 1, divide by 2. */
	/* Toggle OC4A on match, implicit divide by two. */
	TCCR4A = (1 << COM4A0) | ((TIMER4_MODE & 0x3) << WGM40);
	/* Bypass pre-scaler to use 16MHz clock, set upper mode bits. */
	TCCR4B = (1 << CS40) | ((TIMER4_MODE & 0xC) << (WGM42 - 2));
	/* OC4A output is on PH3.
	 * PH4/PWM7/OC4B is the tach opto input -- pull-up active.
	 * PH5/PWM8 drives the tach LED.
	 * PWM9/PH6/OC2B is pulled high as an alternate input
	 * Enable the pins as outputs. */
	PORTH = 0x70;
	DDRH = 0x28;

	/* Timer 5 is the only 16 bit timer with an external clock input.
	 * It is configured in the tach module. */

	return;
}

int main(void)
{
	unsigned tm_100, tm_last_command;
	int i;

	/* We may re-start from a watchdog reset, so we prioritize setting the
	 * outputs to a safe state and enabling status reporting.
	 */
	setup_io_ports();
	setup_uart();

	/* We optionally verify intact firmware before frobbing the traction
	 * circuit.  If the firmware is corrupt, this will never return. */
	check_firmware_integrity();

	show_banner();

	CAN_init();

	/* Persistent config from EEPROM. This may eventually contain
	 * options for configuring the I/O ports, but for now we prefer that
	 * the I/O ports are set up well enough to report errors.
	 */
	read_config();

	/* Read out the pre-start voltages.
	 * We can only do this before the timers start. */
	/* Throw away the first conversions. */
	adc_baseline(-2);
	for (i = 0; i < NUM_ADC_CHANNELS; i++) {
		raw_adc[i] = 	adc_baseline(i);
		raw_adc_sample_time[i] = 0;
	}
	show_adc_voltages();

	/* Initialize the Proportional-Integral control loop. */
	pi_init();

	/* Wait 100mS, recording the number of loops.  We expect about 35960.
	 * This is later used to verify that processor has sufficient idle
	 * time to handle peak workloads. */
	idle_loopcount = wait_time(100);
	serprintf(PSTR("Loops for 100mSec: %d\r\n"), idle_loopcount);

	/* Enable watchdog, which does a reset if we don't feed him.
	 * We must do this before starting any power-control process. */
	watchdog_enable();

	/* Establish baseline quiescent voltages. */
	baseline_m_current = adc_baseline(ADC_CHANNEL_M_CURRENT);
	/* Threshold for a low-read fault is -0.5V down or 1024/10 */
	lowfault_m_current = baseline_m_current - 102;

	/* Set an error flag if any voltages are out of range. */

	setup_timers();
	setup_tach();
	/* The timers must be running for the CAN controller to work! */
	wdt_reset();
	CAN_start();

	/* Initialize timekeeping variables. */
	tm_can_pkt = tm_show_data = tm_100 = get_time();
	tm_last_command = tm_show_data - 10000;

	/* Our operating loop has only non-time-critical code.  Output can
	 * busy-wait on the serial port queue with no preemption, and the CAN
	 * bus controller is polled.
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

		/* Periodically spew a line on the serial port with operating
		 * parameters.	This better done with CAN frames,
		 * but many people are more comfortable with serial data.
		 * Suppress output for a bit if there was recent serial
		 * input.
		 */
		if (config.rtd_period &&
			diff_time(tm_show_data) >= config.rtd_period &&
			diff_time(tm_last_command) > RTD_SILENCE_ON_INPUT) {
			show_serial_data();
			tm_show_data += config.rtd_period;	/* Time for next report. */
			if (tm_show_data - get_time() > 11000)
				tm_show_data = get_time();
			/* Drag-along to avoid false trigger. */
			tm_last_command = get_time() - (10000+1);
		}
		/* Do the 0.1Hz processing. */
		if (diff_time(tm_100) >= 100) {
			tm_100 += 100;		/* Alarm time for next 100msec update. */
			/* This logic should be in a more general event state machine. */
			/* If fault, toggle LED, else light LED */
			if (fault_bits) {
				TOGGLE_LED;
				if (fault_bits & HPL_FAULT) {
					/* High pedal lockout fault set - reset it if
					 * current ref is below threshold */
					if (rt_data.throttle_ref <= HPL_THROTTLE_THRESHOLD) {
						cli(); fault_bits &= ~HPL_FAULT; sei();
					}
				}
				if (fault_bits & PRECHARGE_WAIT) {
					if (precharge_timer == 0) {
						/* Remove precharge wait condition to enable
						 * power to motor */
						cli(); fault_bits &= ~PRECHARGE_WAIT; sei();
					} else {
						/* With half a second left in precharge, close contactor */
						if (precharge_timer <= 5)
							PORTD &= ~PD_CONTACTOR;
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
