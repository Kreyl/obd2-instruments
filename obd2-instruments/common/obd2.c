/* obd2.c: OBD2 CAN interface for the VVVVroom motor controller. */
/*
 * OBD2 interface and reporting.
 * This file implements automotive OBD2, reporting
 * state, status, statistics and setting the configuration.
 *
 * This code currently implements most relevant OBD2 PIDs, with a structure
 * that makes it easy to add more and to report additional values.
 * We respond to SAE J1979 PIDs as well as our own "manufacture specific"
 * PIDs.
 *
 * It combines the OBD2 protocol code with an interface to CAN bus
 * drivers.  This code was previously combined with a MCP2515 CAN
 * controller driver in a single source file.  While that was an
 * awkward mix, it was motivated by the lack of clean layering in
 * CAN.  Using a chip with a different interface or structure of Rx
 * filters and masks propagates changes into the protocol level.
 *
 * There are now multiple CAN controller drivers in separate files, but
 * all are still are intertwined with OBD2 addressing.
 *
 * We default to 500Kbps.  In some configurations we start broadcasting,
 * expecting that other devices will be silently waiting to autobaud
 * against our traffic.
 *
 * References for understanding this code
 *  http://en.wikipedia.org/wiki/On-board_diagnostics
 *  http://en.wikipedia.org/wiki/OBD-II_PIDs
 *  http://en.wikipedia.org/wiki/ISO_15765-2
 *  http://www.semiconductors.bosch.de/pdf/can2spec.pdf
 *  SAE-J2190
 *
 * Notes:
 * DLC (data length) is always 8 with automotive CAN
 *
 * Addresses are poorly considered in CAN.  They are neither source
 * nor destination addresses.  They are abitrarily assigned and not
 * really discoverable.  Pretty much everything dynamic has to be
 * a broadcast, and no 'general purpose CAN stack' is feasible.
 *
 * OBD specifies a 'funcitonal ID' of 0x7DF that acts as a broadcast
 * address, and physical addresses of 0x7E0-7EF.  0x7E0-0x7E7 are for
 * for tester-to-ECU communication, and 0x7E8-0x7EF are for ECU-to-tester.
 * A diag tool will broadcast on 0x7DF, and use the ID computed from the
 * response for subsequent frames with multi-frame messages.  We respond
 * from 0x7E8, thus expect targeted frames at 0x7E0.
 *
 * Internal physical addresses are mostly arbitrary, and in-network
 * communication uses the already-known ID.  We use 0x420, and respond to
 * the same set of queries as with 0x7DF/0x7E8.
 *
 * There must always be another active CAN device on the bus that provides
 * an ACK response to successfully transmit.
 * If using an ELM327 v1.42b and later as the only other bus
 * device, turn silent monitoring off with "AT CSM1".  Otherwise it will
 * only provide the ACK to messages it is listening to.
 */
static const char versionA[] =
"obd2.c: $Id: can-bus.c 155 2011-03-31 00:08:47Z becker $ Copyright Donald Becker\n";

/* We need very few library routines. */
#if defined(STM32)
#include <armduino.h>
extern void *memset(void *s, int c, long n);

#elif defined(SDCC_pic16)
#include <stdint.h>
#include <string.h>
#define inline
#define __attribute__(...) 
#define PGM_P const char *
#define PSTR(str) str
#define prog_uint16_t uint16_t

#elif defined(__AVR)
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(__AVR_ATmega168__)
#include <avr/iom168.h>
#define MEGA168
#elif defined(__AVR_ATmega1280__)
#include <avr/iom1280.h>
#define MEGA1280
#endif

#else
#error "Undefined processor type."
#endif

#if defined(COUGAR)
/* Tweaks to work with the Cougar firmware. */
#include "cougar-export.h"
#include "cougar.h"
#ifndef tach_get_QRPM
#define tach_get_QRPM() 1234*4
#endif
#define use_direct_pwm 0
#endif

#include "vvvvroom.h"
#include "can.h"

/* An abstraction for getting the engine load value. */
extern volatile uint16_t pwm_width;	/* EV controller "throttle plate" 0..511  */
static inline uint8_t engine_load(void)
{
	return pwm_width >> 1;
}

/* We deduce our own clock from counter_1k. */
uint16_t local_clock, tm_seconds;

/* The default VIN, used if not set from the EEPROM config.
 * A VIN is typically only 17 ASCII characters. */
char VIN_str[20] = "WBAAA1305H2320124";
/* Set to run at fixed speed.  Used for cruise control or to run A/C, PS pump
 * etc. */
uint16_t PTO_fast_idle;			/* Commanded motor speed in QRPM. */
/* A single global structure for constructing commands makes it easier to
 * add new responses.
 */
struct CAN_command can_cmd;

/* Our CAN ID for operational (non-diagnostic, higher priority) frames
 * and for OBD2 responses. */
#define CAN_PHY_ADDR CAN_SID(0x420)
#define CAN_ECU_ID CAN_SID(0x7e8)

/* Internal state and structures. */
uint8_t CAN_reset_done = 0;
uint8_t CAN_enabled = 0;
uint8_t can_verbose = 0;

/* Multi-frame (ISO-TP 15765-4) response state.
 * The VIN request was the first multi-line response I wrote, so
 * the state continues to use that name.
 */
struct CAN_multiframe {
	int16_t msg_cnt;		 /* If non-zero, more remains to be sent. */
	char *message;			 /* Current message pointer. */
	uint8_t frame_idx;		 /* Frame index for header. */
	uint8_t delay, window;			/* Info from flow control. */
	uint16_t last_send;				/* Use for flow control delay. */
	uint16_t id;					/* Usually the ECU ID. */
};
#if defined(SDCC_pic16)
static struct CAN_multiframe VIN = {0, };
#else
static struct CAN_multiframe VIN = {msg_cnt:0, };
#endif

/* Not used -- better known as the numbers than the name.
 * Here for reference only. */
enum CAN_modes {
	ShowCurrentData=1, ShowFreezeData=2,
	ShowStoredDiagCodes=3, ClearDiagCodes=4,
	TestResultsO2=5, TestResults=6,
	ShowDiagErrors=7, CANControl=8, VehicleInfo=9, PermanentDTC=10,
};

/* Out tenative "manufacture-specific" commands. */
enum CAN_Commands {
	CAN_PI_PARAM, CAN_THROTTLE_RANGE,
};

#if defined(COUGAR_SETVAR)
/* A list of the operating variables that may be read and set. */
struct canned_response {
	unsigned int *var1, *var2;
	unsigned char flags;
	unsigned char pad;
} can_msg_list[] = {
	{(unsigned *)&config.Kp, (unsigned *)&config.Ki, 0},	/* CAN_PI_PARAM: */
	{&config.throttle_min_raw_counts, &config.throttle_min_raw_counts, 0},
	{&config.throttle_fault_raw_counts, 0, 0},
	{&config.throttle_pos_gain, &config.throttle_pwm_gain, 0},
	{(unsigned *)&config.current_ramp_rate, 0, 0},
	{&config.pwm_filter, 0, 0},
	{&config.motor_os_th, &config.motor_os_ft, 0},
	{&config.motor_os_dt, &config.pwm_deadzone, 0},
	{&config.motor_sc_amps, 0, 0},
	{&config.battery_amps_limit, 0, 0},
	{&config.precharge_time, 0 /* Precharge percent */, 0},
};
/*
 * This will become the manufacturer-specific section.
 * We will respond with our state variables using the table above.
 */
void can_state(void)
{
	int var1, var2;
	unsigned char var3;
	switch (msg_type) {
	case CAN_PI_PARAM:
		var1 = config.Kp;
		var2 = config.Ki;
		break;
	case CAN_THROTTLE_RANGE:
		var1 = config.throttle_min_raw_counts;
		var2 = config.throttle_max_raw_counts;
		break;
	default:
		break;
	}
}

#endif

#if 0
/* Temperature normalizaiton calibration using a linear interpolation table.
 * The table is build using calibration numbers taken at a few temperatures,
 * including an estimate for the extreme lower and upper raw values.
 */
struct {int raw, C; } temp_table[] = {
	{0, -50},					/* Fake a lower bound: raw_adc of 0 is -50C */
	{400, 0},					/* Measured value of ice at 0C */
	{500, 20},					/* Comfy room at 20C */
	{700, 100},					/* Measured value of boiling water at 100C */
	{1024, 200},				/* Estimate upper bound: raw_adc of 1024 200C */
};
int raw_to_calibrated(int adc_raw)
{
	int i;
	int celsius;

	/* Scan the table, knowing that we can't fall off the end. */
	for (i = 1; adc_raw < temp_table[i].raw; i++)
		;
	/* i is now the index for the higher value. Interpolate.  */
	celsius = (adc_raw - temp_table[i-1].raw)*temp_table[i].C +
		(temp_table[i-1].raw - adc_raw)*temp_table[i-1].C;
	celsius /= temp_table[i].raw - temp_table[i-1].raw;
	return celsius;
}
#endif


/* This is the initially interesting part: respond to standard OBD2
 * messages.  To keep the code simple, the surrounding code deals with
 * addressing and message format.
 */
uint8_t OBD2_respond(void)
{
	uint8_t count = 0;

	switch(can_cmd.mode) {
	case 1: switch(can_cmd.pid) {		/* ShowCurrentData */
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0xB8;	  	/* 01 -- 03 04 05 -- -- -- */
			can_cmd.dataB = 0x13;		/* -- -- -- 0c -- -- 0f 10 */
			can_cmd.dataC = 0x80; 		/* 11 -- -- -- -- -- -- -- */
			can_cmd.dataD = 0x13;		/* -- -- -- 1c -- -- 1f 20 */
			count = 6;
			break;
		case 0x01:
			can_cmd.dataA = fault.bits ? 0x81 : 0;
			can_cmd.dataB = 0x00;	/* Nothing to see here.  Move along now. */
			can_cmd.dataC = 0x00;	/* No tests available. */
			can_cmd.dataD = 0x00;
			count = 6;
			break;
		case 0x03:
			/* Report closed loop when the current sensor can be read. */
			can_cmd.dataA = use_direct_pwm ? 8:2;
			can_cmd.dataB = 0;	/* with no second fuel system */
			count = 4;
			break;
		case 0x04:	/* Calculated Engine load 0..255. */
			/* Percentage of instantaneous available torque, not rated torque.
			 * On the QAR controller we report PWM percentage, although
			 * amps is the best estimate of torque. */
			can_cmd.dataA = engine_load();
			count = 3;
			break;
		case 0x05:	/* Coolant temp A-40 -40C..+215C, we use heatsink temp. */
#if 0
			/* Assume a LM335 where  Kelvin = 100 * 5V * (count / 1024)
			 * Our baseline temp is -40C or 233.15 Kelvin, or 2.33V
			 * which is an A/D reading of about 477
			 * DataA = ((500 * rt_data.raw_hs_temp)/ 1024) - 233  */
			can_cmd.dataA = (125 * (rt_data.raw_hs_temp - 477)) >> 8;
#else		/* We are using using a MCP9701 sensor on this channel. */
			/* MCP9701 temp sensor conversion. */
			can_cmd.dataA = (rt_data.raw_hs_temp + 78)>>2;
#endif
			count = 3;
			break;
		case 0x0C:		/* Engine RPM, revs per 4 minutes.  */
			{
				uint16_t qrpm = tach_get_QRPM();
				can_cmd.dataA = qrpm >> 8;
				can_cmd.dataB = qrpm;
				count = 4;
			}
			break;
		case 0x0F:	/* Intake temp A-40, -40C..+215C, use second temp sensor. */
			/* The calculation is for a MCP9701 sensor.
			 * It outputs 400mV at 0C, and increases by 19.5mV/C. */
			if (raw_adc[3] < 1000) {			/* No report if missing. */
#if 0											/* MCP9701 sensor */
				can_cmd.dataA = (raw_adc[3]+78)>>2;
#else				/* Ooops, use LM335 conversion instead. */
				can_cmd.dataA = (125 * (raw_adc[3] - 477)) >> 8;
#endif
				count = 3;
			}
			break;
		case 0x10: {
			/* Nominally MAF (air flow) in 0.01 gram/sec.  We instead report
			 * loosely-calibrated amps, largely because it is a standard
			 * ScanGauge meter. */
			uint16_t amps = current_fb * 100;
			can_cmd.dataA = amps >> 8;
			can_cmd.dataB = amps;
			count = 4;
			break;
		}
		case 0x11:	/* Absolute throttle pos. 0..255, we use normalized
					 * pedal position. */
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			count = 3;
			break;
		case 0x1C:
			can_cmd.dataA = 9;	/* We respond to EOBD, OBD and OBD II */
			count = 3;
			break;
		case 0x1E:
			can_cmd.dataA = PTO_fast_idle ? 1 : 0;
			count = 2;
			break;
		case 0x1F: {		/* Run time since start.  */
			can_cmd.dataA = tm_seconds >> 8;
			can_cmd.dataB = tm_seconds;
			count = 4;
			break;
		}
		case 0x20:				/* PID 21-40 support, hand calculated. */
			can_cmd.dataA = 0x00;
			can_cmd.dataB = 0x00;
			can_cmd.dataC = 0x00;
			can_cmd.dataD = 0x01; 		/* Only 0x40 */
			count = 6;
			break;
		case 0x40:				/* PID 41-60 support, hand calculated. */
			can_cmd.dataA = 0x0A;		/* -- -- -- -- 45 -- 47 -- */
			can_cmd.dataB = 0x00;		/* None */
			can_cmd.dataC = 0x80;		/* 51 -- -- --  -- -- -- -- */
			can_cmd.dataD = 0x10; 		/* 59: -- -- -- 5C  -- -- -- --   */
			count = 6;
			break;
#if 0
		case 0x42: {
			/* Control module voltage: 12V expected, we report 5V supply. */
			uint16_t voltage;
			/* Measure the 1.1V ref, round a bit to allow exactly 5.000V */
			voltage = 1125000L / rt_data.raw_1_1V;
			can_cmd.dataA = voltage >> 8;
			can_cmd.dataB = voltage;
			break;
		}
#endif
		case 0x45:				/* Relative throttle position. */
		case 0x47:				/* Absolute throttle position B. */
		case 0x4C:				/* Commanded throttle. */
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			break;
		case 0x51:
			can_cmd.dataA = 8;	/* Running on 'lectricity! */
			count = 3;
			break;
		case 0x5C:	/* Engine oil temp A-40, -40C..+215C, report motor temp. */
			/* The calculation is for a MCP9701 sensor.
			 * It outputs 400mV at 0C, and increases by 19.5mV/C. */
			if (raw_adc[6] < 1000)
				can_cmd.dataA = (raw_adc[6]+78)>>2;
			else		/* Report 0C if sensor missing. */
				can_cmd.dataA = 40;
			count = 3;
			break;
			/* A few more are easily added once we have these tested. */
			/* 0x4D Run time with MIL on. */
			/* 0x4E Time since trouble codes cleared. */
			/* 0x5A relative accelerator pedal position. */
			/* 0x5B hybrid pack remaining life. */
			/* 0x5E fuel rate. */
			/* 0x5F emissions requirements for this vehicle. */
		}
		break;
	case 2: switch(can_cmd.pid) {
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0x40;	/* Hand calculated. */
			can_cmd.dataB = 0x00;
			can_cmd.dataC = 0x00;
			can_cmd.dataD = 0x00;
			count = 6;
			break;
		case 0x02:				 /* Freeze data at last DTC  */
			can_cmd.dataA = 0x00;		/* No DTC reported. */
			can_cmd.dataB = 0x00;
			count = 4;
			break;
		}
		break;
	case 3: {
		/* Return a list Diagnostic Trouble Codes (DTCs)
		 * P0110-4 Intake air circuit malfunctionn
		 * P0115-9 Engine coolant temperature
		 * P0120-4 Thottle position sensor/switch (TPS)
		 * P0195-9 Engine oil temperature sensor
		 * +0  Circuit malfunction
		 * +1  Circuit range (e.g. gaps in response) problem
		 * +2/+3/+4  high/low/intermittent input
		 * With more than 3 DTCs, we must contruct an ISO-TP sequence.
		 * See the VIN response for an example.  For now we keep it simple
		 * and never have a multiple frame response.
		 */
		can_cmd.mode = 0x01;	  	/* First DTC byte 0 and 1 */
		can_cmd.pid = 0x20;
		can_cmd.dataA = 0x01;		/* Start of second DTC */
		count = 2;
		return count;				/* Avoid munging the PID field. */
	}
	case 9: switch(can_cmd.pid) {
			/* The VIN and calibration ID are the only times we need
			 * multi-frame messages, thus we shift the header rather
			 * than have general-purpose code.  */
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0x40;	  	/* 01: -- 02 -- -- -- -- -- -- */
			can_cmd.dataB = 0x00;		/* 09: -- -- -- -- -- -- -- -- */
			can_cmd.dataC = 0x00; 		/* 11: -- -- -- -- -- -- -- -- */
			can_cmd.dataD = 0x00;		/* 19: -- -- -- -- -- -- -- -- */
			count = 6;
			break;
		case 0x02:
			can_cmd.mode = 20;		  	/* Bytes in response. */
			can_cmd.pid = 0x49;			/* Shift the mode and PID over. */
			can_cmd.dataA = 0x02;
			VIN.delay = 255;			/* Mark as wait for flow control. */
			VIN.frame_idx = 1;			/* First of 5 lines */
			can_cmd.dataB = 1;			/* Always 1: Only a single VIN */
			VIN.msg_cnt = 17;			/* Strlen(VIN_str). */
			VIN.message = VIN_str;
			/* Only 3 bytes in the first response */
			for (count = 0; count < 3; count++)
				((char *)&(can_cmd.dataC))[count] = *VIN.message++;
			VIN.msg_cnt -= count;
			count = 20;
			break;
		}
		break;
#if ! defined(COUGAR)
		/* Mode A0 -- our vendor-specific/proprietary parameter set.
		 * We generate a response message that includes the original command.
		 */
	case 0xA0: switch(can_cmd.pid) {
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0x3C;	  	/* 01: -- -- 03 04 05 06 -- -- */
			can_cmd.dataB = 0x1F;		/* 09: -- -- -- 0C 0D 0E 0F 10 */
			can_cmd.dataC = 0xFF; 		/* 11: 11 12 13 14 15 16 17 18 */
			can_cmd.dataD = 0xFE;		/* 19: 19 1A 1B 1C 1D 1E 1F -- */
			count = 6;
			break;
		case 0x03:		/* Remote throttle position.  */
			if (can_cmd.cnt == 4) {
				throttle.remote = can_cmd.dataA;
				throttle.remote_valid = can_cmd.dataB;
				throttle.remote_time = clock_1msec;
				count = 4;
			}
			break;
		case 0x04:		/* Fault bits.  */
			can_cmd.dataA = fault.bits;
			can_cmd.dataB = 0;			/* All upper fault bits are zero. */
			count = 4;
			break;
		case 0x05:		/* Max motor current.  */
			can_cmd.dataA = m_current_peak >> 8;
			can_cmd.dataB = m_current_peak;
			count = 4;
			break;
		case 0x06:
			can_cmd.dataA = fault.cycles_off >> 8;
			can_cmd.dataB = fault.cycles_off;
			count = 4;
			break;
		case 0x0C:		/* Set target engine RPM, in quarter-RPMs.  */
			if (can_cmd.cnt == 4 && can_cmd.dataA < 0x60) {
				PTO_fast_idle = (can_cmd.dataA << 8) + can_cmd.dataB;
				tach_target_qrpm = PTO_fast_idle;
				count = 4;
			}
			break;
		case 0x0D:		/* Set maximum RPM, in quarter-RPMs.  */
			if (can_cmd.cnt == 4 && can_cmd.dataA < 0x60) {
				tach_redline_qrpm = (can_cmd.dataA << 8) + can_cmd.dataB;
				count = 4;
			}
			break;
		case 0x0E:		/* Get target RPM, quarter-RPMs.  */
			can_cmd.dataA = tach_target_qrpm >> 8;
			can_cmd.dataB = tach_target_qrpm & 0xFF;
			count = 4;
			break;
		case 0x0F:		/* Get maximum RPM, quarter-RPMs.  */
			can_cmd.dataA = tach_redline_qrpm >> 8;
			can_cmd.dataB = tach_redline_qrpm & 0xFF;
			count = 4;
			break;
		default:
			if (can_cmd.pid >= 0x10 && can_cmd.pid < 0x20) {
				/* Read raw sensor value.  */
				uint8_t adc_channel = can_cmd.pid - 0x10;
				can_cmd.dataA = raw_adc[adc_channel] >> 8;
				can_cmd.dataB = raw_adc[adc_channel] & 0xFF;
				count = 4;
			}
			break;
		}
		break;
#endif
	}
	if (count) {
		can_cmd.cnt = count;
		can_cmd.mode |= 0x40;
	} else {					/* mode $7F: reject message */
		can_cmd.dataB = 0x11;	/* Unsupported mode */
		can_cmd.dataA = can_cmd.pid;
		can_cmd.pid = can_cmd.mode;
		can_cmd.mode = 0x7F;
		can_cmd.cnt = 4;
	}
	return count;
}

int8_t CAN_process_rx_frame(void)
{
	uint8_t count;

	if (can_cmd.cnt < 7) {		/* The normal case, a Single Frame */
		count = OBD2_respond();
		if (count) {
			/* Most responses are simple Single Frames.  Long responses
			 * have filled in the flow control structure. */
			if (count >= 8) {
				can_cmd.cnt = 0x10;		/* 0x10 | (count>>8) */
				can_cmd.mode = count;
				VIN.id = CAN_ECU_ID;
			}
			can_cmd.id = CAN_ECU_ID;
			return CAN_dev_transmit();
		}
	} else if ((can_cmd.cnt & 0xF0) == 0x30) { /* A flow control frame. */
		switch (can_cmd.cnt & 0x0F) {
		case 0:
			VIN.window = can_cmd.mode;
			VIN.delay = can_cmd.pid;
			/* VIN.last_send = counter_10k; */
			break;
		case 1:							/* Wait */
			VIN.delay = 255;
			break;
		case 2:	default:				/* Overflow/invalid: abort */
			VIN.msg_cnt = 0;
			break;
		}
	}
	return 0;
}

/* Reset and initialize the interface.
 * Synchronize if possible, but do not start reception.
 * Returns non-zero if the interface does not exist or is borked.
 * Call early to overlap any CAN reset with other device init.
 */
int8_t CAN_init(void)
{
	tm_seconds = 0;
	local_clock = clock_1msec;
	CAN_reset_done = CAN_enabled = 0;

	/* This will be cleaned up after development stabilizes. */
	if (CAN_dev_init() != 0)
		return 1;
	CAN_reset_done = 1;
	return 0;
}

/* Start the interface and enable frame reception.
 */
int8_t CAN_start(void)
{
	int8_t status;
	if (! CAN_reset_done)
		CAN_init();
	status = CAN_dev_start();
	if (status == 0)
		CAN_enabled = 1;
	return status;
}

/* Poll the CAN bus.
 * This is called as frequently as possible from the main loop.
 * It may be used as the main interactive loop or for deferred interrupt
 * processing.
 */
void CAN_poll(void)
{

	if ( ! CAN_enabled && CAN_start() != 0)
		return;

#if defined(COUGAR)
	/* Keep our local OBD time in seconds.
	 * Robust and non-critical, so do not bother locking.
	 */
	if (counter_1k - local_clock > 976) {
		tm_seconds++;
		local_clock += 976;
	}
#endif

	CAN_dev_poll();

	if (VIN.msg_cnt != 0 && VIN.delay != 255) {
		/* More multi-frame (only VIN!) lines remain to be sent. */
		uint8_t i;
#if 0
		/* Check delay, if the timer is running. */
		if (timer_10KHz &&			/* Timer must be running */
			(VIN.delay <= 127 &&
			 timer_10KHz - VIN.last_send > VIN.delay*10) ||
			(VIN.delay > 0xF0 && VIN.delay <= 0xF9 &&
			 timer_10KHz - VIN.last_send > VIN.delay - 0xF0 + 1))
#endif
			can_cmd.cnt = 0x20 | (VIN.frame_idx & 0x0F);
		for (i = 0; i < 7  && --VIN.msg_cnt > 0; i++)
			((char *)&(can_cmd.mode))[i] = *VIN.message++;
		can_cmd.id = CAN_ECU_ID;
		if (CAN_dev_transmit() != 0) {
			VIN.message -= i;	/* Revert changes */
			VIN.msg_cnt += i;
		} else {
			VIN.frame_idx++;
		}
	}

	return;
}

/* This is periodically called from the main loop to report operating
 * conditions, as well as to generate traffic for monitors that expect
 * to autobaud.
 */
void CAN_heartbeat(void)
{
	if ( ! CAN_enabled)
		return;

	can_cmd.cnt = 0x07;
	can_cmd.mode = 0x10;		/* Our own packet type. */
	can_cmd.pid = 0x23;
	can_cmd.dataA = rt_data.throttle_ref >> 1; 	/* Throttle position */
	can_cmd.dataB = current_fb>>2;
	can_cmd.dataC = tach_get_QRPM() >> 7;		/* Scaled RPM, 0-8K. */
	can_cmd.dataD = (125 * (rt_data.raw_hs_temp - 477)) >> 8;
	can_cmd.unnamed = fault.bits; 			/* Longer than OBD2 responses. */
	can_cmd.id = CAN_PHY_ADDR;
	CAN_dev_transmit();
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
