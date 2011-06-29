/* serial-commands.c: Commands accepted on the serial interface. */
/*
 * Tables for command-plus.c.
 * The commands are of two types: set a variable, and call a function.
 *
 * The first is done with a table listing 16 bit unsigned variables.
 * The second is done with a similar table of function pointers.
 */

/* Tables of variable names that may be interactively set.
 */

#include <avr/pgmspace.h>
#include "vvvvroom.h"
#include "command.h"

static void volts(uint16_t val)
{
	/* We should not call show_adc_voltages() while the PI loop is running
	 * on the AVR, as it disturbs the slow conversion in progress.
	 * But we can with the STM32. */
	show_adc_voltages();
	return;
}

static void idle(uint16_t val)
{
	serprintf(PSTR("AVR %d%% idle\r\n"),
			  (unsigned)(wait_time(100) * (unsigned long)100 / idle_loopcount));
}
static void uptime(uint16_t val)
{
	serprintf(PSTR("Uptime %d seconds\r\n"), clock_1msec / 1000);
}
	
static void restart(uint16_t val)
{
	watchdog_enable();
	while(1)					/* Test the watchdog */
		;
}

struct cmd_var_entry const cmd_var_table[] = {
	{"config.throttle_min_raw_counts", &config.throttle_min_raw_counts, 0,1023},
	{"config.throttle_max_raw_counts", &config.throttle_max_raw_counts, 0,1023},
	{"throttle_fault_raw", &config.throttle_fault_raw_counts, 0, 0xFFFF},
	{"config.throttle_pos_gain", &config.throttle_pos_gain, 0, 0xFFFF},
	{"config.throttle_pwm_gain", &config.throttle_pwm_gain, 0, 0xFFFF},
	{"config.current_ramp_rate", &config.current_ramp_rate, 0, 0xFFFF},
	{"config.pwm_filter", &config.pwm_filter, 0, 7},
	{"motor_overspeed_threshold", &config.motor_os_th, 0, 0xFFFF},
	{"config.motor_os_ft", &config.motor_os_ft, 0, 9999},
	{"config.motor_os_dt", &config.motor_os_dt, 0, 99},
	{"config.pwm_deadzone", &config.pwm_deadzone, 0, 0xFFFF},
	{"config.motor_sc_amps", &config.motor_sc_amps, 0, 0xFFFF},
	{"config.battery_amps_limit", &config.battery_amps_limit, 0, 0xFFFF},
	{"rtd_period", &config.rtd_period, 0, 30000},
	{"rp", &config.rtd_period, 0, 30000},
	{"current", &m_current_peak, 0, 0}, /* Clear on write only  */
	{"pwm", &use_direct_pwm, 0, 1000},
	{0, 0, 0, 0},
};


struct cmd_func_entry const cmd_func_table[] = {
#if 0
	{"config", show_config, 0, 0},
	{"save", write_config, 0, 0},
	{"reset", mcp2515_reset, 0, 0},
	{"rpm", send_rpm_message, 0, 0},
	{"time", time, 0, 0},
	{"verbose", set_verbose, 0, 10},
#endif
	{"idle", idle, 0, 0xFFFF},
	{"restart", restart, 0, 0},
	{"uptime", uptime, 0, 0},
	{"volts", volts, 0, 0xFFFF},
	{0, 0, 0, 0},
};



/* Our preferred reporting is with CAN
 * but many people are more comfortable with serial data.
 * Modify this to report in the format you prefer.
 */
void show_serial_data(void)
{
	serprintf(PSTR("Faults %4x Instrument data 0: %d 1: %d\r\n"),
			  fault.bits, raw_adc[0]);
}


/*
 * Local variables:
 *  compile-command: "make serial-commands.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
