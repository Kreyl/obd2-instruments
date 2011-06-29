#ifndef _VVVVROOM_H
#define _VVVVROOM_H
/* vvvvroom.h: Vvvvroom automotive instrument cluster definition file. */
/*
	$Revision: 1.1 $ $Date: 2011/5/31 00:00:21 $
	Hardware compatibility settings.
	This file provides symbolic names and macros to mask the difference
	between hardware implementations.
	It allows a single source base to support different processor, sensor
	and instrument configurations without too much ugliness in the
	procedual code.

	Written 2010,2011 by Donald Becker and William Carlson
	This software may be used and distributed according to the terms
	of the GNU General Public License (GPL), incorporated herein by
	reference.  Firmware interacting with these functions are derivative
	works and thus are covered the GPL.  They must include an explicit
	GPL notice and follow the terms of the license.

	Other contributers:
	<currently none>
*/


/* Primary controller clock (Oscillator) in cycles per second.
 * This name is used by the AVR header files for baudrate and delay.
 */
#if !defined(F_CPU) || F_CPU == 0
#define F_CPU 16000000
#endif

/* Time in msec to stop serial status output on keystroke. */
#define RTD_SILENCE_ON_INPUT 5000

/* Some symbolic pin definitions.
 * We can only be a little hardware-independent, as we must write
 * to the specific byte-wide port.  We put the port letter in the
 * symbolic name to indicate this. */

#if defined(MEGA1280)
#define PB_LED (1 << PB7)			/* Arduino Mega on-board LED */
#define PD_CONTACTOR (1 << PD7)		/* Contactor Opto LED, low to light */
#define PD_OC_CLEAR (1 << PD6)		/* Low clears an OverCurrent fault. N/C */
#elif defined(COUGAR)
#define PB_PWM (1 << PB1)			/* PWM output pin (high to turn FETs on)*/
#define PINB_OC_STATE (1 << PINB0)	/* OverCurrent state (high means fault) */
#define PB_OC_CLEAR (1 << PB2)		/* Low clears an OverCurrent fault. */
#define PD_LED (1 << PD6)			/* Cougar Idle LED, high to light LED */
/*
 * Port C on the AVR controls the analog inputs.
 * When an analog input is used the pins must be inputs, weak pullups off.
 */
#define PC_ANALOGS_USED ((1 << PC0) | (1 << PC1) | (1 << PC2))
#endif

/* The number of A/D channels that are active and should be read in the
 * normal scan.
 * We normally read all and sort out the meaning elsewhere.
 * Most values change either far more slowly (e.g. temperature) or too
 * fast to sample (e.g. waveforms) */
#define NUM_ADC_CHANNELS 16

/* ADC channels.  The default names are for the VVVVroom EV motor controller.
 * Rename as necessary.  */
enum {
	ADC_CHANNEL_THROTTLE=0,			/* Throttle input. */
	ADC_CHANNEL_TEMP=1,				/* Heatsink temperature */
	ADC_CHANNEL_M_CURRENT=2,		/* Motor current */
	ADCChan_B_Current=3, 			/* Battery current. */
	ADCChan_HSTemp2=4,				/* Heaksink #2 temperature. */
	ADCChan_HSTemp3=5,				/* Heaksink #3 temperature. */
	ADCChan_MotorTemp=6, 			/* Motor case temperature. */
	/* The following are traction voltage referenced. */
	ADCChan_B_Volts=8,				/* Traction battery and cap rail voltage */
	ADCChan_C_Volts=9,				/* (0-200V by 0.2V, 10 bits minimum) */
	ADCChan_M_Volts=11,				/* Output rail average voltage. */
	ADCChan_M_Drive_Voltage=12,		/* Drive voltage peak.  */
	ADCChan_M_Freewheel_Voltage=13,	/* Freewheel voltage peak.  */
	ADCChan_C_Ripple=10,			/* Capacitor rail ripple.
									 * (0-20V by 0.1V, 8 bits minimum) */
	ADCChan_D1_Volts=14,			/* Gate driver voltage+ */
	ADCChan_D2_Volts=15,			/* Gate driver voltage- */
};

/* Program and EEPROM memory space constants. */
#define PROGSTART 0x0000	  /* Program start address */
/* Address in EEPROM of our persistent configuration tables. */
#define EE_CONFIG_ADDRESS 0	  /* Base address of config in EEPROM. */
#define EE_CONFIG_COPIES 4	  /* Duplicate copies of config in EEPROM */

/* Number of cycles (milliseconds) the throttle may out of range before a
 * a fault is set. */
#define THROTTLE_FAULT_COUNTS 200

/* High pedal lockout threshold, using the normalized [0..511] throttle.
 * This avoid false triggers, but allows minor creep-forward at start-up. */
#define HPL_THROTTLE_THRESHOLD 2

/* The PI loop variables have somewhat arbitrary scale.
 * We want to use fixed-point math with reasonable precision and no chance of
 * overflow.
 * The Cougar firmware uses a 0..511 range because they started with the 9
 * bit timer mode.  It's a reasonable choice.  Lacking any solid reason to do
 * otherwise, we continue with that here.
 */
#define MAX_CURRENT_REF 511

/* Overspeed is detected by an abnormally low motor current for the PWM
 * perecentage.  This is the number of cycles (milliseconds) before an
 * overspeed fault is set.
 */
#define MOTOR_OS_DETECT_TIME 10

/* A somewhat arbitrary number for thermal cut-back when using a LM335.
 * Change if using a different sensor.
 * The LM335 is inexpensive and has an output voltage in "Kelvin",
 * 2.73V at 0C, 2.98V at 25C.
 *
 * An alternate is the LM35: a bit more expensive, but requires no bias
 * resistor, uses less current, and has a lower impedance thus a better
 * A/D conversion.
 *
 * Assume a LM335.  75C = 3.48V, 3.48V/5.00V * 1023 = 712
 *   125C = 3.98V, 3.98V/5.00V * 1023 = 815
 */
#define THERMAL_CUTBACK_START 712
int16_t overheat;				/* A positive value indicates overheat */

/* Cougar firmware included an option to use a 8KHz PWM signal.
 * We temporarily are including the same option to see how it impacts
 * heat and noise.  Most of the code is configured to allow an arbitrary
 * PWM, constrained by the need to sample the motor current in the middle
 * of the on "push" pulse.
 */
#if defined(USE_PWM8K)			/* Here just for documentation. */
#define PWM8K
#endif

/* The possible faults are bitmapped into a byte. */
#define THROTTLE_FAULT (1 << 0)
#define VREF_FAULT (1 << 1)
#define PRECHARGE_WAIT (1 << 5)
#define MOTOR_OS_FAULT (1 << 6)
#define HPL_FAULT (1 << 7)

#define INITIAL_FAULTS HPL_FAULT
#define fault_bits fault.bits

extern struct fault_info {
	volatile unsigned char bits;
	unsigned cycles_off;		/* PWM cycles with 0 output cause by faults. */
	unsigned pi_overwork;
	unsigned motor_overspeed_fault_count;
} fault;

/* A union to pack/extract bytes or 16 bit words from 32 bit values. */
typedef union {
	uint8_t  b[2];
	uint16_t w[1];
} packW_t;
typedef union {
	uint8_t  b[4];
	uint16_t w[2];
	uint32_t l;
} packL_t;

/* Prototype for vvvvroom.c */
void write_config(void);
unsigned long wait_time(unsigned howlong);
void watchdog_disable(void);
void watchdog_enable(void);
/* Prototypes for atmega-dev.c */
void start_adc_conversion(int8_t channel);
int read_adc_voltage(void);
uint16_t adc_baseline(int8_t channel);
void show_adc_voltages(void);

/* Prototypes for serial.c */ 
extern volatile unsigned long serial_txbytes;
extern volatile unsigned long serial_rxbytes;
/* Get character from input FIFO, return -1 if empty. */
int uart_getchar(void);
/* Put character to the output, return -1 if failed-full. */
unsigned char uart_putchar(char c);
/* Configure the UART. */
void setup_uart(void);

/* Simplified version of printf() in printf.c */
extern int serprintf(const char *format, ...)
	__attribute__ ((format(printf, 1, 2)));;

/* Prototypes for pi-loop.c */
void pi_init(void);
void pi_loop(void);

/* Prototypes for command.c */
void show_config(unsigned mask);
void do_serial_port_char(unsigned char c);
void show_serial_data(void);

/* Prototypes for spi-adc.c */
int adc_setup(void);
int adc_convert(char channel);
int adc_convert9(char channel);

/* Prototypes for tach.c */
void setup_tach(void);
uint16_t tach_get_QRPM(void);
void tach_tick(uint8_t counter_8k, uint8_t edge);
uint8_t tach_last_period_high;
uint16_t tach_last_period;

volatile unsigned raw_adc[NUM_ADC_CHANNELS];
volatile unsigned raw_adc_sample_time[NUM_ADC_CHANNELS];

/* Loops per 100msec when not executing the control and monitoring loop.
 * Used for calculating load in operation. */
unsigned long idle_loopcount;

/* Global timekeeping section:
 * Most subsystems use a 1KHz clock synchronized with the PWM timer.
 * The overflow carry is counted in time_high for periods over 60 seconds
 * (ODB2 reports the total run time). 
 */
extern volatile uint16_t clock_1msec; 	/* Exact 1KHz timer. */
extern volatile unsigned counter_1k; 	/* 1KHz (976Hz to be exact) counter */
extern unsigned time_high;
extern unsigned tm_show_data;	 /* Timestamp of recent data log line. */

/* Pulse width percentage, the pulse width in a 9 bit, [0..511] range.
 * This is directly the timer compare register value in mode three, or
 * scaled in other modes.
 * Consider changing this to 8 bits or 12 bits. */
extern volatile unsigned pwm_width;
extern volatile unsigned pwm_sr_width;		/* Sync rectification setting */

extern unsigned pwm_deadtime;		/* Timer ticks between lo/hi gate on. */

#if 0
extern unsigned ocr1a_lpf;	/* ocr1a run through lowpass filter (sort of averaged) */
extern unsigned long ocr1a_lpf_32; // ocr1a low pass filter sum
#endif


/* Motor current sensor: Baseline (quiescent, 0 amp) reading and low fault. */
extern unsigned baseline_m_current;
extern unsigned lowfault_m_current;
extern uint16_t use_direct_pwm;

extern unsigned tach_redline_qrpm;			/* Redline in QRPM */
extern unsigned tach_rpm_min_period;		/* Tach period at redline  */
extern unsigned tach_target_qrpm;			/* Target speed in QRPM */

extern int throttle_ref;		/* Reference (commanded) throttle position. */
extern int current_ref;			/* Reference (desired) current. */
extern int current_fb;			/* Current feedback, actual current. */
/* Temperature-limited maximum current. */
extern unsigned max_current_ref;
extern uint16_t m_current_peak;


extern unsigned battery_amps;	/* Calculated instantaneous battery current. */
extern unsigned long battery_ah;	/* Calculated battery mAH consumed. */

extern unsigned long bat_amp_lim_510; // battery amps limit multiplied by 510 (max PWM)
extern unsigned long motor_overspeed_threshold;
/* Motor overspeed fault timer in milliseconds. */
extern unsigned motor_os_fault_timer;
/* Motor overspeed debounce timer in milliseconds. */
extern unsigned char motor_os_count;
extern unsigned throttle_fault_counts;
extern unsigned long motor_overspeed_fault_count;

/* The motor PI control loop parameters came from the Cougar motor
 * controller so that we had a starting point.
 *
 * I also used the state structure pretty much unchanged so that we
 * could print idential real-time reports on the serial port and use the
 * existing display tools.
 */
typedef struct {
	int throttle_ref;
	int current_ref;
	int current_fb;
	unsigned raw_hs_temp;
	unsigned raw_throttle;
	unsigned battery_amps;
	unsigned long battery_ah;
} realtime_data_type;

realtime_data_type rt_data;

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
} throttle;

/* How many timer_1k ticks before a CAN remote throttle command times out. */
#define THROTTLE_REMOTE_TIMEOUT 1000

/* The persistent configuration parameters stored in the EEPROM. */
typedef struct {
	unsigned magic;					// Use 0x12ab to be compatible w/ Cougar
	/* PI loop proportional and integral gains.  The Kp value in EEPROM will
	 * be multiplied by 1024 to get K1, while the K2 value will be
	 * Ki - Kp*1024.
	 */
	int Kp;
	int Ki;
	/* Throttle thresholds, in A/D converter units.  Note that Cougar used
	 * inverted voltage, 5V-0V, while we use the 0-5V standard,
	 * 0V = closed throttle, 5V = pedal down / go fast. */
	unsigned throttle_min_raw_counts;
	unsigned throttle_max_raw_counts;
	unsigned throttle_fault_raw_counts; /* After 200mS this sets a fault. */
	/* The throttle pedal has a fixed gain setting, not proportional to
	 * configured range. */
	unsigned throttle_pos_gain;			// gain for actual throttle position
	unsigned throttle_pwm_gain;			// Feedback gain for PWM (voltage)
	/* Cap on current (amps) change per msec. */
	uint16_t current_ramp_rate;
	/* Real Time Data reporting period, in msec. */
	unsigned rtd_period;
	/* Index for PWM time-average function. */
	unsigned pwm_filter;
	unsigned motor_os_th;				// motor overspeed threshold
	unsigned motor_os_ft;				// motor overspeed fault time
	unsigned motor_os_dt;				// motor overspeed detect time
	/* Minimum period to turn on the FETs.  Short on pulses generate heat
	 * with minimal output power. */
	unsigned pwm_deadzone;
	/* Battery amps limit */
	unsigned battery_amps_limit;
	unsigned precharge_time;			// precharge time in 0.1 second increments
	unsigned motor_sc_amps;				// motor current must be > motor_sc_amps to calculate motor speed
	/* Non-Cougar config settings.  Space for five 16 bit words. */
	unsigned motor_amps_limit;		// Force PWM=0 when m_current>limit
	unsigned spares[4];					// space for future use
	unsigned crc;						// checksum for verification
} __attribute__((packed)) config_type;

extern config_type config;

#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
