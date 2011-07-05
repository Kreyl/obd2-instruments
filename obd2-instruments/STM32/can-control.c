/* can-control.c: Send CAN messages to the vvvvroom motor controller. */
/*
	Written 2010 by Donald Becker and William Carlson.

	The original authors may be reached as
	donald.becker@gmail.com
	Annapolis MD 21403

	This software is released under the Gnu Public License (GPL) v2.
	Commercial licenses are available you do not wish to comply with
	the GPL terms.
	Support information and updates are available from the authors.
*/
static const char versionA[] =
"$Id: can-control.c 139 2011-03-19 00:33:15Z becker $ Copyright Donald Becker\n";

#if defined(STM32)
#include "armduino.h"
#include "vvvvroom.h"

#else
#if defined(__AVR_ATmega168__)
#define MEGA168
#elif defined(__AVR_ATmega1280__)
#define MEGA1280
#endif

#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(MEGA168)
#include <avr/iom168.h>
#else
#include <avr/iom1280.h>
#endif
#endif

#include "can.h"
#include "command.h"
#include "vvvvroom.h"

uint16_t heartbeat_period = 0;		/* mSec between CAN info transmit. */

config_type config;				/* Must declare, for now. */

/* Duplicate of the mcp2515.c SPI code, here for test commands. */
#if defined(STM32)
#define SPI_CAN_CS 15		/* Abitrary digital pin used for active-low /CS */
#define GPIO_SPI_BRR GPIOA_BRR
#define GPIO_SPI_BSRR GPIOA_BSRR
#define CAN_CS_ENABLE \
	GPIO_SPI_BRR = (1 << SPI_CAN_CS);		/* Chip select low */
#define CAN_CS_DISABLE \
	GPIO_SPI_BSRR = (1 << SPI_CAN_CS);		/* Chip select high */
#define SPI_CLK_DIV SPI_BRdiv4
#define SPDR	SPI1_DR
#define SPSR	SPI1_SR
#define SPI_CR1 SPI1_CR1
#define SPI_CR2 SPI1_CR2
#define SPI_Transmit(data)  SPDR = data; while( ! (SPSR & SPI_RXNE)) ; SPDR;
#else
#define PB_CAN_CS DDB0		/* Abitrary digital pin used for active-low /CS */
#define CAN_CS_ENABLE	PORTB &= ~(1 << PB_CAN_CS);		/* Chip select low */
#define CAN_CS_DISABLE	PORTB |= (1 << PB_CAN_CS);		/* Chip select high */
#define SPI_CLK_DIV	(0 << SPR0) /* 0,1,2,3 is divisor by 4,16,64,128  */
#define SPI_Transmit(data)  SPDR = data; while( ! (SPSR & (1<<SPIF))) ; SPDR;
#endif

volatile uint16_t raw_adc[NUM_ADC_CHANNELS];
volatile unsigned raw_adc_sample_time[NUM_ADC_CHANNELS];

uint16_t misc, qrpm;

/* CAN OBD2. */

/* Internal settings. */
#define F_CPU 16000000					/* 16MHz AVR CPU clock. */

/* Motor controller parameters that we have to publish to can-bus.c
 * Not all group neatly into structures.
 */
struct fault_info fault = {
 bits: 0,
};
config_type PROGMEM default_config = {
	 motor_amps_limit: 60,
};
struct throttle_params throttle;
realtime_data_type rt_data;
volatile unsigned counter_1k; 	/* 1KHz (1 msec) counter */
unsigned time_high;
/* EV controller "throttle plate" 0..511  */
volatile uint16_t pwm_width = 42;
int current_fb = 0;
uint16_t use_direct_pwm = 100;
extern uint16_t PTO_fast_idle;			/* Commanded motor speed in QRPM. */
unsigned tach_redline_qrpm;			/* Redline in QRPM */
unsigned tach_target_qrpm;			/* Target speed in QRPM */
uint16_t m_current_peak;

/* Fake the interface function for tachometer info. */
uint16_t tach_get_QRPM(void)
{
	return qrpm;
}

/* Timer 1 compare/capture interrupt handler.
 * The real time monitoring happens in this function.
 */
ISR(TIM1_UP_TIM16)
{
	/* Clear the interrupt source. */
	TIM1_SR = 0;
	return;
}

inline unsigned get_time(void)
{
	return clock_1msec;
}

#if ! defined(STM32)
/* Configure the I/O port functions and directions.
 * We set the timer for a 1KHz interrupt.
 */
void setup_io_ports(void)
{

	/* Port B signals:
	 * On the Mega 1280, PB1/2/3 is SPI port: SCK/MOSI/MISO, PB7 is the LED
	 * Set the other output pins on PB to high / weak pullups. */
	PORTB = 0xff;
	DDRB = 0xe7;

	/* Port C
	 * On the Mega all Port C pins are GPIO.
	 */

	/* On the ATMega168 and later the digital input buffers must be
	 * disabled when the pins are used for the A/D converter.
	 */
	/* We enable all analog inputs, thus disable all digital buffers. pg296 */
	DIDR0 = 0xff;				/* ADC0-7 */
	DIDR2 = 0xff;				/* ADC8-15.  Note: not DIDR1 ! */

	/* Port D
	 * On the Arduino Mega 1280
	 * PD0-PD4 are used for communication.
	 */

#if defined(__AVR_ATmega1280__)
	/* The USART and ADC are on at reset, but should be enabled by clearing
	 * their bits in the PRR Power Reduction Register.
	 */
	PRR0 &= ~(PRADC | PRUSART0 | PRTIM1);
	/* And we must enable the UART0 Rx/Tx pins as outputs. */
	PORTE = 0x02;
	DDRE = 0x02;
#endif

	/* Timer channels.
	 * Configure timer channel 1 as a /1600 timer to get a 10KHz interrupt.
	 */
#define TIMER_MODE 12
#define TIMER_FREQ 10000		/* Timer at 10 KHz */
	TCNT1 = 0;					/* Load 16 bit counter 1 */
	ICR1 = (long)F_CPU / TIMER_FREQ;
	/* Let counter 1 run at FCPU (16MHz), reset to 0 at ICR1 */
	TCCR1A = ((TIMER_MODE & 0x3) << WGM10);			/* No outputs enabled. */
	TCCR1B = (1 << CS10) | ((TIMER_MODE & 0xC) << (WGM12 - 2));
	/* Enable input capture 1 interrupt  pg 168. */
	TIMSK1 = (1 << ICIE1);

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
	return;
}

/* A stub to keep functions unchanged from their operational version. */
inline void wdt_reset(void)
{
	return;
}

/* Our base value for ADCSRA ADC Status Register A.
 * Set the clock prescaler to /128, 125KHz. */
const uint8_t adcsra_base = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
void start_adc_conversion(int8_t channel)
{
	if (channel > 7) {			/* Select the high ADC channels. */
		ADCSRB = (1 << MUX5);
		ADMUX = ADMUX = (01 << REFS0) | (channel & 0x07);
	} else {
		ADCSRB = 0;
		ADMUX = ADMUX = (01 << REFS0) | (channel & 0x1F);
	}
	/* Start a one-shop ADC conversion pg. 277 */
	ADCSRA = adcsra_base | (1 << ADSC);
}

/* Read the results of the most recent conversion. */
static inline unsigned read_adc_voltage(void)
{
	while (ADCSRA & (1 << ADSC))
		;
	/* Use the assembler short-cut to read the register pair in order. */
	return ADC;
}
	
/* Establish a baseline value for measured operating conditions by averaging
 * over 16 samples. */
unsigned int adc_baseline(int8_t channel)
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
#endif

#if 0
/* Show the voltages on the internal ADC.
 * Note that this is strictly start-up debugging.  It can only be called
 * before the timer is running and triggering A/D conversions.  */
void show_adc_voltages(void)
{
	int8_t i;
	int count;
	uint16_t voltage;

	/* Convert the internal voltage reference (AVR=1.1V, STM:1.2V) and
	 * inverting to calculate Vref+ (which is Vcc). */
	count = adc_baseline(16);

	serprintf(PSTR("System voltages\r\n"));
	serprintf(PSTR("STM Vcc %dmV (%4d)\r\n"),
			  1200*4096 / count, count);

	for (i = -2; i < 16; i++) {
		start_adc_conversion(i);
		wdt_reset();
		count = read_adc_voltage();
		voltage = (625*(long)count) >> 7;
		serprintf(PSTR("ADC%2d %4d 	%4dmV\r\n"), i, count, voltage);
	}
	return;
}
#endif

/* Command-line commands. */
static void help(uint16_t val)
{
	int i;
	for (i = 0; cmd_func_table[i].name; i++) {
		serprintf(PSTR(" %s"), cmd_func_table[i].name);
	}
	serprintf(PSTR("\n"));
	return;
}

static void cmd_time(uint16_t val)
{
	serprintf(PSTR("Uptime %5d msec\r\n"), get_time());
	return;
}

static void set_pwm1(uint16_t val)
{
	if (val < 9999) {
		TIM3_CCR3 = val;
		TIM3_CCR4 = 333;
		TIM3_EGR = 1;
	}
}

static void set_pwm2(uint16_t val)
{
	if (val < 9999) {
		TIM4_CCR1 = val;
		TIM4_CCR2 = 333;
		TIM4_EGR = 1;
	}
}

/* Set the TIM3_CH2 output, the second one down on the QAR board.
 * This is broken.  Due to a chip bug, the output stays at '1'.  The
 * pin can only be used for input or GPIO.  */
static void set_pwm3(uint16_t val)
{
	if (val < 29999)
		TIM3_CCR2 = val;
}

static void volts(uint16_t val)
{
	int i;
	/* The PI motor control loop runs the A/D converters.  We must not
	 * use the ADC if it is running, but need to gather our own
	 * updated values if it is not.
	 */
	for (i = 0; i < NUM_ADC_CHANNELS; i++) {
		uint16_t voltage = (13*raw_adc[i]) >> 4;
		serprintf(PSTR("STM ADC%2d %4d %d.%3dV\r\n"),
				  i, raw_adc[i], voltage / 1000, voltage % 1000);
	}
	return;
}

/* A debugging command to send all zeros on the SPI MOSI line. */
void mosi_zero(uint16_t val)
{
	int i;
	CAN_CS_ENABLE;
	for (i = 0; i < 400000; i++) {
		uint8_t status; /* = can_get_status(); */
		SPDR = 0x0F;
		while( ! (SPSR & SPI_RXNE))
			;
		status = SPDR;
	}
	CAN_CS_DISABLE;
}

static void can_reset(uint16_t val)
{
	CAN_init();
}

static void can_stop(uint16_t val)
{
	CAN_dev_stop();
}

static void cmd_init(uint16_t val)
{
	CAN_dev_init();
	return;
}

#if 0
void mcp2515_reset(uint16_t val)
{
	uint16_t t0, tdiff;
	int i;

	t0 = get_time();
	serprintf(PSTR("Reset started at %d msec\r\n"), t0);
	i = CAN_dev_reset();				/* Takes quite some time. */

#if 1
	tdiff = get_time() - t0;
	serprintf(PSTR("Reset took %d usec\r\n"), tdiff);
#endif

	/* Wait until a read status command works.
	 * This typically takes 4 loops after a reset, so allow 10x.*/
	for (i = 0; i < 4000; i++) {
		uint8_t status; /* = can_get_status(); */
		CAN_CS_ENABLE;
		SPI_Transmit(0xA0);			 /* MCP_ReadStatus */
		status = SPI_Transmit(0xF0);		/* 0xFF: detect non-response. */
		CAN_CS_DISABLE;
		if (status != 0xF0) {
			serprintf(PSTR("Reset status was %2x\r\n"), status);
			break;
		}
	}

	tdiff = get_time() - t0;
	serprintf(PSTR("Reset took %d loops, %d0 usec, %d to %d \r\n"),
			  i, tdiff, t0, get_time());
	return;
}

void mcp2515_init(uint16_t val)
{
	CAN_dev_init();
	return;
}
void mcp2515_sleep(uint16_t val)
{
	CAN_CS_ENABLE;
	SPI_Transmit(0x02);			/* MCP_Write */
	SPI_Transmit(0x0F);			/* CANCTRL */
	SPI_Transmit(val == -1 ? 0x20 : val);		/* Set sleep mode. */
	CAN_CS_DISABLE;

	return;
}
void mcp2515_status(uint16_t val)
{
	CAN_show_registers(-1);
	return;
}
void mcp2515_leds(uint16_t val)
{
	mcp2515_set_pins((val<<4) | 0x0C);
}
void mcp2515_gppins(uint16_t val)
{
	uint8_t pins = mcp2515_get_pins();
	serprintf(PSTR("Input pins %c %c %c\r\n"),
			  (pins & 0x08) ? '1' : '0',
			  (pins & 0x10) ? '1' : '0',
			  (pins & 0x20) ? '1' : '0');
}
#endif

void show_regs(uint16_t val)
{
	if (val != 0xFFFF) {
		CAN_show_registers(val & 0xF0);
		return;
	}
	CAN_show_registers(val);
	return;
}

void set_verbose(uint16_t val)
{
	can_verbose = val;
	return;
}

void can_heartbeat(uint16_t val)
{
	CAN_heartbeat();
}	

/* Send a set-RPM message to the motor controller. */
void send_rpm_message(uint16_t val)
{
	can_cmd.cnt = 0x04;
	can_cmd.mode = 0x10;
	can_cmd.pid = 0x42;
	can_cmd.dataA = val >> 8;
	can_cmd.dataB = val;
	can_cmd.id = CAN_SID(0x420);
	CAN_dev_transmit();
	return;
}

struct cmd_var_entry const cmd_var_table[] = {
	{"hp", &heartbeat_period, 0, 10000},
	{"misc", &misc, 0,1023},
	{"pwm", (uint16_t *)&pwm_width, 0, 1000},
	{"qrpm", &qrpm, 0,0xFFFF},
	{0, 0, 0, 0},
};

struct cmd_func_entry const cmd_func_table[] = {
	{"heartbeat", can_heartbeat, 0, 0},
	{"help", help, 0, 0},
	{"init", cmd_init, 0, 0},
#if 0
	{"leds", mcp2515_leds, 0, 0},
	{"pins", mcp2515_gppins, 0, 0},
#endif
	{"pwm1", set_pwm1, 0, 0},
	{"pwm2", set_pwm2, 0, 0},
	{"pwm3", set_pwm3, 0, 25000},
	{"regs", show_regs, 0, 0},
#if 0
	{"sleep", mcp2515_sleep, 0, 0},
	{"status", mcp2515_status, 0, 0},
#endif
	{"reset", can_reset, 0, 0},
	{"rpm", send_rpm_message, 0, 0},
	{"stop", can_stop, 0, 0},
	{"time", cmd_time, 0, 0},
	{"verbose", set_verbose, 0, 10},
	{"volts", volts, 0, 0xFFFF},
	{"zero", mosi_zero, 0, 0xFFFF},
	{0, 0, 0, 0},
};


/* Timestamps for recent actions or events. */
unsigned tm_can_pkt;					/* Timer for CAN status heartbeat. */
unsigned tm_last_command, tm_prev_heartbeat;

int main(void)
{
	/* We may re-start from a watchdog reset, so we prioritize setting the
	 * outputs to a safe state and enabling status reporting.
	 */
	setup_io_ports();
	setup_uart();
	serprintf(PSTR("\nCAN interaction console\n"));

	setup_clock();
	setup_timers();

	if (can_verbose)
		serprintf(PSTR("Initializing the CAN controller.\r\n"));
	{
		int result, i = 0;
		SPDR = 0xC0;  /* MCP_Reset */
		while( ! (SPSR & SPI_RXNE) && i < 1000)
			i++;
		result = SPDR;
		serprintf("Reset returned %2x after %d cycles.\n", result, i);
	}

#if 0
	GPIOC_BSRR = 0x00000300;		/* LED on. */
	mosi_zero(-1);
	GPIOC_BSRR = 0x03000000;		/* LED off. */
	mosi_zero(-1);
	mosi_zero(-1);
	GPIOC_BSRR = 0x00000300;		/* LED on. */
	serprintf("Finished MOSI zero cycle.\n");
#endif

	raw_adc[0] = 100;
	raw_adc[1] = 101;
	raw_adc[2] = 102;
	raw_adc[3] = 122;
	rt_data.raw_hs_temp = 230;
	rt_data.throttle_ref = 300;

	CAN_init();
	serprintf("Finished CAN reset.\n");
	CAN_start();
	serprintf("Finished CAN startup.\n> ");

	/* Initialize the timekeeping variables. */
	tm_can_pkt = get_time();

	/* Our operating loop is mostly non-time-critical code.
	 * The interesting stuff happens in the interrupt handlers.
	 */
	while (1) {
		int c;

		/* See if we have any communication waiting. */
		c = uart_getchar();
		if (c >= 0) {
			do_serial_port_char((unsigned char)c);
			tm_last_command = get_time();
		}

		CAN_poll();			/* Poll the CAN bus controller. */

		if (heartbeat_period &&
			get_time() - tm_prev_heartbeat >= heartbeat_period) {
			rt_data.throttle_ref+=2;
			CAN_heartbeat();
			tm_prev_heartbeat = get_time() + 1000;
		}

		/* Do any 1 Hz processing. */
		if (get_time() > tm_can_pkt) {
#if 1
			/* Cycle the LED brightness to indicate liveness. */
			if (CAN_enabled)
				TIM4_CCR1 = (1<<((tm_can_pkt/1000) & 0x7));
#endif
			tm_can_pkt = get_time() + 1000;
			/* The QAR board has the LED on PB0 */
			GPIOB_ODR = PORTB ^ 1;
		}
	}
	return(0);
}

/*
 * Local variables:
 *  compile-command: "make can-control"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
