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
#warning "This source code version omits the CAN SPI interface."
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
 * The timekeeping and real time monitoring happens in this function.
 */
volatile uint16_t clock_1msec;
volatile uint16_t clock_high;		/* If we ever need more than 65K seconds. */
ISR(SIG_INPUT_CAPTURE1)
{
	static int phase = 0;
	if (++phase < 10)
		return;
	phase = 0;
	clock_1msec++;
	if (clock_1msec == 0) {
		clock_high++;
	}
	return;
}

inline unsigned get_time(void)
{
	unsigned tsec;
	cli(); tsec = clock_1msec; sei();
	return tsec;
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
#endif

/* A stub to keep functions unchanged from their operational version. */
inline void wdt_reset(void)
{
	return;
}

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

static void cmd_vars(uint16_t val)
{
	int i;
	serprintf(PSTR("Variable  Value\n"));
	for (i = 0; cmd_var_table[i].name; i++) {
		serprintf(PSTR(" %s = %d\n"),
				  cmd_var_table[i].name, *cmd_var_table[i].ptr);
	}
	return;
}

static void cmd_time(uint16_t val)
{
	serprintf(PSTR("Uptime %5d msec\r\n"), get_time());
	return;
}

static void volts(uint16_t val)
{
	int i;
	/* If the PI motor control loop is running, it controls the A/D converter
	 * and updates raw_adc[].  If it's not running we need to gather our own
	 * updated values.
	 */
	for (i = 0; i < NUM_ADC_CHANNELS; i++) {
		uint16_t voltage = (13*raw_adc[i]) >> 4;
		serprintf(PSTR("AVR ADC%2d %4d %d.%3dV\r\n"),
				  i, raw_adc[i], voltage / 1000, voltage % 1000);
	}
	return;
}

void cmd_init(uint16_t val)
{
	CAN_dev_init();
	return;
}

void cmd_start(uint16_t val)
{
	CAN_dev_start();
	return;
}

void mcp2515_reset(uint16_t val)
{
	uint16_t t0, tdiff;
	int i;

	t0 = get_time();
	serprintf(PSTR("Reset started at %d msec\r\n"), t0);
	i = CAN_dev_init();				/* Takes quite some time. */

	tdiff = get_time() - t0;
	serprintf(PSTR("Reset took %d usec\r\n"), tdiff);

	/* Wait until a read status command works.
	 * This typically takes 4 loops after a reset, so allow 10x.*/
	for (i = 0; i < 4000; i++) {
		uint8_t status;
		CAN_CS_ENABLE;
		SPI_Transmit(0xA0);			 /* MCP_ReadStatus */
		status = SPI_Transmit(0xFF);		/* 0xFF: detect non-response. */
		CAN_CS_DISABLE;
		if (status != 0xFF) {
			serprintf(PSTR("Reset status was %2x\r\n"), status);
			break;
		}
	}

	tdiff = get_time() - t0;
	serprintf(PSTR("Reset took %d loops, %d0 usec, %d to %d \r\n"),
			  i, tdiff, t0, get_time());
	return;
}

static void can_reset(uint16_t val)
{
	mcp2515_reset(val);
}

static void can_stop(uint16_t val)
{
	mcp2515_sleep(-1);	/* Could be CAN_dev_stop(); */
}

/* Named 'sleep', because it defaults to setting the chip to sleep mode.
 * It may be used for more general mode setting.
 * Use 'sleep 64' to set loopback mode, or 'sleep 96' for listen-only mode.
 */
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

void show_regs(uint16_t val)
{
	int i;
	if (val != 0xFFFF) {
		CAN_show_registers(val & 0xF0);
		return;
	}
	for (i = 0; i < 0x80; i += 0x10)
		CAN_show_registers(i);
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
	if (val > (6250*4))
		val = 0;
	can_cmd.cnt = 0x04;
	can_cmd.mode = 0x10;
	can_cmd.pid = 0x42;
	can_cmd.dataA = val >> 8;
	can_cmd.dataB = val;
	can_cmd.dataC = clock_1msec >> 8;
	can_cmd.dataD = clock_1msec;
	can_cmd.id = CAN_SID(0x420);
	CAN_dev_transmit();
	return;
}

struct cmd_var_entry const cmd_var_table[] = {
	{"hp", &heartbeat_period, 0, 10000},
	{"misc", &misc, 0,1023},
	{"pwm", (uint16_t *)&pwm_width, 0, 1000},
	{"qrpm", &qrpm, 0,0xFFFF},
	{"verbose", (uint16_t *)&can_verbose, 0, 10},
	{0, 0, 0, 0},
};

struct cmd_func_entry const cmd_func_table[] = {
	{"heartbeat", can_heartbeat, 0, 0},
	{"help", help, 0, 0},
	{"init", cmd_init, 0, 0},
	{"leds", mcp2515_leds, 0, 0},
	{"pins", mcp2515_gppins, 0, 0},
	{"regs", show_regs, 0, 0},
	{"reset", can_reset, 0, 0},
	{"rpm", send_rpm_message, 0, 0},
	{"sleep", mcp2515_sleep, 0, 0},
	{"start", cmd_start, 0, 0},
	{"status", mcp2515_status, 0, 0},
	{"stop", can_stop, 0, 0},
	{"time", cmd_time, 0, 0},
	{"v", set_verbose, 0, 10},
	{"vars", cmd_vars, 0, 10},
	{"volts", volts, 0, 0xFFFF},
	{0, 0, 0, 0},
};


/* Timestamps for recent actions or events. */
unsigned tm_can_pkt;					/* Timer for CAN status heartbeat. */
unsigned can_beat_period = 1000;		/* Period between CAN heartbeats. */
unsigned tm_last_command, tm_prev_heartbeat;

int main(void)
{
	/* We may re-start from a watchdog reset, so we prioritize setting the
	 * outputs to a safe state and enabling status reporting.
	 */
	setup_io_ports();
	setup_uart();
	serprintf(PSTR("\nCAN interaction console\n"));

	/* The timers must be running for the CAN controller to work. */
#if 0						/* Don't.  We need to verify initialization. */
	if (can_verbose)
		uart_print(PSTR("Initializing the CAN controller.\r\n"));
	can_setup();
#else
	DDRB |= ((1<<DDB2)|(1<<DDB1)|(1<<PB_CAN_CS));
	CAN_CS_DISABLE;
	/* Turn on: SPI enable, MSB first, 0,0, Master, F_OSC/4. */
	SPCR = (1 << SPE) | (1 << MSTR) | SPI_CLK_DIV;
	SPSR = 1 << SPI2X;				/* Double clock to 8MHz */
#endif

	/* Initialize the timekeeping variables. */
	tm_can_pkt = get_time();
	serprintf(PSTR("> "));

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
#if 0
			/* Toggle LED.  The QAR board has the LED on PB0, Mega on... */
			GPIOB_ODR = PORTB ^ 1;
#endif
			tm_can_pkt = get_time() + 1000;
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
