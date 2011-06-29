/* cougar-export.h: Prototypes that cougar.h should have. */
/* This file contains the cougar-specific headers for can-bus.c.
 * These really should have been in cougar.h.
 */

#ifndef _COUGAR_EXPORT_H
#define _COUGAR_EXPORT_H

extern volatile unsigned char fault_bits;
extern char uart_str[];
extern volatile unsigned counter_1k;		//  Actually 976Hz
extern volatile unsigned char counter_16k;
extern volatile unsigned char counter_8k;

void u16_to_str(char *str, unsigned val, unsigned char digits);
void u16x_to_str(char *str, unsigned val, unsigned char digits);

typedef struct {
	unsigned magic;						// must be 0x12ab
	int Kp;								// PI loop proportional gain
	int Ki;								// PI loop integreal gain
	unsigned throttle_min_raw_counts;	// throttle low voltage (pedal to metal)
	unsigned throttle_max_raw_counts;	// throttle high voltage (foot off pedal)
	unsigned throttle_fault_raw_counts;	// throttle fault voltage (after 200mS)
	unsigned throttle_pos_gain;			// gain for actual throttle position
	unsigned throttle_pwm_gain;			// gain for pwm (voltage)
	int current_ramp_rate;				// current ramp rate
	unsigned rtd_period;				// real time data period
	unsigned pwm_filter;				// filter for ocr1a_lpf
	unsigned motor_os_th;				// motor overspeed threshold
	unsigned motor_os_ft;				// motor overspeed fault time
	unsigned motor_os_dt;				// motor overspeed detect time
	unsigned pwm_deadzone;				// pwm deadzone (before FETs start to conduct)
	unsigned battery_amps_limit;		// battery amps limit
	unsigned precharge_time;			// precharge time in 0.1 second increments
	unsigned motor_sc_amps;				// motor current must be > motor_sc_amps to calculate motor speed
	unsigned spares[5];					// space for future use
	unsigned crc;						// checksum for verification
} config_type;

typedef struct {
	int throttle_ref;
	int current_ref;
	int current_fb;
	unsigned raw_hs_temp;
	unsigned raw_throttle;
	unsigned battery_amps;
	unsigned long battery_ah;
} realtime_data_type;

config_type config;
realtime_data_type rt_data;

#endif

/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
