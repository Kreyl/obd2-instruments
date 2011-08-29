#ifndef _VVVVROOM_H
#define _VVVVROOM_H
/* vvvvroom.h: Vvvvroom motor controller definition file. */
/*
	$Revision: 1.0 $ $Date: 2011/1/22 00:00:21 $
	Hardware compatibility settings.
	This file provides symbolic names and macros to mask the difference
	between hardware implementations.
	It is designed primarily to allow a single source base to support
	several different motor controller designs and microcontroller chips.

	Written 2010 by Donald Becker and William Carlson
	This software may be used and distributed according to the terms
	of the GNU General Public License (GPL), incorporated herein by
	reference.  Firmware interacting with these functions are derivative
	works and thus are covered the GPL.  They must include an explicit
	GPL notice and follow the terms of the license.

	Other contributers:
	<currently none>
*/


#if defined(STM32)
#include <armduino.h>
#endif

/* Msec to stop status output on keystroke. */
#define RTD_SILENCE_ON_INPUT 5000

/* The limit on PWM count when pulse width is proportional to the throttle. */
#define PROPORTIONAL_PWM_MAX 300

/* The number of cycles between low and high side devices
 * being enabled.  This varies based on the PWM clock, isolation delay,
 * the gate drive power and drive device.
 * Zero is allowed if the gate driver handles the deadtime.
 */
#define PWM_SR_DEADTIME 10



/* Some symbolic pin definitions.
 * We can only be a little hardware-independent, as we must write
 * to the specific GPIO port.  We put the port letter in the
 * symbolic name to indicate this.
 */
#define PD_CONTACTOR (1 << 7)		/* Contactor coil drive, low to turn on. */

/* Set the SPI port to communicate with the MCP2515 CAN controller. */
#if defined(STM32)
/* The active-low CS can be on an abitrary GPIO pin.  */
#define SPI_CAN_CS 15
#define GPIO_SPI_BRR GPIOA_BRR
#define GPIO_SPI_BSRR GPIOA_BSRR
#define CAN_CS_ENABLE \
	GPIO_SPI_BRR = (1 << SPI_CAN_CS);		/* Chip select low */
#define CAN_CS_DISABLE \
	GPIO_SPI_BSRR = (1 << SPI_CAN_CS);		/* Chip select high */
/* The SPI data clock is <=10MHz.  Set divisor based on the peripheral clock. */
#define SPI_CLK_DIV SPI_BRdiv4
/* Use SPI1 on the VLDiscovery board. */
#define SPDR	SPI1_DR
#define SPSR	SPI1_SR
#define SPI_CR1 SPI1_CR1
#define SPI_CR2 SPI1_CR2
#endif

#if defined(STM32)
#define ADC_VOLTAGE_SCALE 104
#define NUM_ADC_CHANNELS 18
#else
#define ADC_VOLTAGE_SCALE 625
#define NUM_ADC_CHANNELS 16
#endif

extern volatile uint16_t raw_adc[NUM_ADC_CHANNELS];
extern volatile unsigned raw_adc_sample_time[NUM_ADC_CHANNELS];

/* The mapping of the A/D converter channels.
 * Additional channels are only on our controller. */
enum {
	ADC_CHANNEL_THROTTLE=5,		/* Throttle input. */
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

/* Number of milliseconds the throttle may out of range before a
 * a fault is set. */
#define THROTTLE_FAULT_COUNTS 200

/* High pedal lockout threshold, using the normalized [0..511] throttle. */
#define HPL_THROTTLE_THRESHOLD 2

/* The PI loop variables have somewhat arbitrary scale.
 * We want to use fixed-point math with reasonable precision and no chance of
 * overflow.
 * The Cougar firmware uses a 0..511 range because they started with the 9
 * bit timer mode.  It's a reasonable choice.  Lacking any solid reason to do
 * otherwise, we continue with that here.
 */
#define MAX_CURRENT_REF 511

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
extern int16_t overheat;			/* A positive value indicates overheat */

/* The possible faults are bitmapped into a byte. */
enum {
	THROTTLE_FAULT = (1 << 0), VREF_FAULT = (1 << 1),
	PRECHARGE_WAIT = (1 << 5), MOTOR_OS_FAULT = (1 << 6), HPL_FAULT = (1 << 7)
};

#if 0
extern volatile unsigned char fault_bits;
#else
#define fault_bits fault.bits
#endif

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
uint32_t wait_time(unsigned howlong);
void watchdog_disable(void);
void watchdog_enable(void);
void show_adc_voltages(void);

/* Board/chip-specific functions, in their own file. */
int check_firmware_integrity(void);
void read_config(void);
void write_config(void);
void wdt_reset(void);

extern volatile uint32_t clock_1msec;
void setup_clock(void);
void setup_io_ports(void);
void setup_timers(void);
int adc_setup(void);
void start_adc_conversion(int8_t channel);
int read_adc_voltage(void);
unsigned int adc_baseline(int8_t channel);


/* Prototypes for serial.c */ 
extern volatile uint32_t serial_txbytes;
extern volatile uint32_t serial_rxbytes;
/* Get character from input FIFO, return -1 if empty. */
int uart_getchar(void);
/* Put character to the output, return -1 if failed-full. */
unsigned char uart_putchar(char c);
/* Configure the UART. */
void setup_uart(void);

/* Prototypes for pi-loop.c */
void pi_init(void);
void pi_loop(unsigned raw_m_current, unsigned raw_throttle);

/* Prototypes for command.c */
void show_banner(void);
void show_config(unsigned mask);
void do_serial_port_char(unsigned char c);
void show_cougar_rt_data(void);

/* Prototypes for spi-adc.c */
int adc_setup(void);
int adc_convert(char channel);
int adc_convert9(char channel);

/* Prototypes for tach.c */
void setup_tach(void);
uint16_t tach_get_QRPM(void);
void tach_tick(uint8_t counter_8k, uint8_t edge);
extern uint8_t tach_last_period_high;
extern uint16_t tach_last_period;

/* Loops per 100msec when not executing the control and monitoring loop.
 * Used for calculating load in operation. */
extern uint32_t idle_loopcount;

/* Global timekeeping section:
 * We need a 1msec clock for CAN flow control, and 1 second clock with
 * 16 bits of run time for ODB2.
 * The AVR controller uses a 1KHz clock synchronized with the PWM timer,
 * the overflow carry counted in time_high for periods over 60 seconds.
 * The STM32 uses the 32 bit Systick timer configured for 1msec ticks.
 */
extern volatile uint32_t clock_1msec;

/* Pulse width percentage, the pulse width in a 9 bit, [0..511] range.
 * This is directly the timer compare register value in mode three, or
 * scaled in other modes.
 * Consider changing this to 8 bits or 12 bits. */
extern volatile uint16_t pwm_width;
extern volatile uint16_t pwm_sr_width;		/* Sync rectification setting */

extern uint8_t pwm_deadtime;		/* Timer ticks between lo/hi gate on. */

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
extern uint32_t battery_ah;	/* Calculated battery mAH consumed. */

extern uint32_t bat_amp_lim_510; // battery amps limit multiplied by 510 (max PWM)
extern uint32_t motor_overspeed_threshold;
/* Motor overspeed fault timer in milliseconds. */
extern unsigned motor_os_fault_timer;
/* Motor overspeed debounce timer in milliseconds. */
extern unsigned char motor_os_count;
extern unsigned throttle_fault_counts;
extern uint32_t motor_overspeed_fault_count;

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
	uint32_t battery_ah;
} realtime_data_type;

extern realtime_data_type rt_data;

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
	uint16_t throttle_min_raw_counts;
	uint16_t throttle_max_raw_counts;
	uint16_t throttle_fault_raw_counts; /* After 200mS this sets a fault. */
	/* The throttle pedal has a fixed gain setting, not proportional to
	 * configured range. */
	uint16_t throttle_pos_gain;			// gain for actual throttle position
	uint16_t throttle_pwm_gain;			// Feedback gain for PWM (voltage)
	/* Cap on current (amps) change per msec. */
	uint16_t current_ramp_rate;
	/* Real Time Data reporting period, in msec. */
	uint16_t rtd_period;
	/* Index for PWM time-average function. */
	uint16_t pwm_filter;
	uint16_t motor_os_th;				// motor overspeed threshold
	uint16_t motor_os_ft;				// motor overspeed fault time
	uint16_t motor_os_dt;				// motor overspeed detect time
	/* Minimum period to turn on the FETs.  Short on pulses generate heat
	 * with minimal output power. */
	uint16_t pwm_deadzone;
	/* Battery amps limit */
	uint16_t battery_amps_limit;
	uint16_t precharge_time;			// precharge time in 0.1 second increments
	uint16_t motor_sc_amps;				// motor current must be > motor_sc_amps to calculate motor speed
	/* Non-Cougar config settings.  Space for five 16 bit words. */
	uint16_t motor_amps_limit;		// Force PWM=0 when m_current>limit
	uint16_t spares[4];					// space for future use
	uint16_t crc;						// checksum for verification
} __attribute__((packed)) config_type;

extern config_type config;
extern const config_type default_config;

#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
