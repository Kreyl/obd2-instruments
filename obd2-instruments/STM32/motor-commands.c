/* motor-commands.c: Command line commands for the VVVVroom motor controller. */
/*
 * Tables for command-plus.c.
 * The commands are of two types: set a variable, and call a function.
 *
 * The first is done with a table listing 16 bit unsigned variables.
 * The second is done with a similar table of function pointers.
 */

/* Tables of variable names that may be interactively set.
 */

#include <armduino.h>
#include "command.h"
#include "vvvvroom.h"


static void volts(uint16_t val)
{
	int i;
	/* We should not call show_adc_voltages() while the PI loop is running
	 * on the AVR, as it disturbs the slow conversion in progress.
	 * But we can with the STM32. */
	for (i = 0; i < NUM_ADC_CHANNELS; i++) {
		uint16_t voltage = (13*raw_adc[i]) >> 4;
		serprintf(PSTR("STM ADC%2d %4d %d.%3dV\n"),
				  i, raw_adc[i], voltage / 1000, voltage % 1000);
	}
	return;
}

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

static void idle(uint16_t val)
{
	serprintf(PSTR("CPU %d%% idle\n"),
			  (unsigned)(wait_time(100) * (unsigned long)100 / idle_loopcount));
}

static void uptime(uint16_t val)
{
	serprintf(PSTR("Uptime %d seconds\n"),
			  clock_1msec / 1000);
}
	
static void restart(uint16_t val)
{
	int i;
	watchdog_enable();
	for (i = 10000; i; i--)
		;					/* Test the watchdog */
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
	{"help", help, 0, 0},
	{"idle", idle, 0, 0xFFFF},
	{"restart", restart, 0, 0},
	{"uptime", uptime, 0, 0},
	{"vars", cmd_vars, 0, 0xFFFF},
	{"volts", volts, 0, 0xFFFF},
	{0, 0, 0, 0},
};

/* The Cougar controller had a mode to periodically spew a line
 * with operating parameters.  Our preferred reporting is with CAN,
 * but there are several display apps for the Cougar serial data format.
 * Our code for doing this much simpler since we have printf().
 */
int raw_throttle_max, raw_throttle_min;
int motor_pwm_min, motor_pwm_max;

void show_cougar_rt_data(void)
{
	serprintf(PSTR("TR=%4d CR=%4d CF=%3d PW=%3d HS=%4d RT=%4d "
				   "FB=%x BA=%d AH=%d.\n"),
			  rt_data.throttle_ref, rt_data.current_ref, rt_data.current_fb,
			  pwm_width, rt_data.raw_hs_temp, rt_data.raw_throttle, fault.bits,
			  rt_data.battery_amps, rt_data.battery_ah);
	serprintf(PSTR(" Throttle bounds: %4d-%4d  PWM range%3d-%3d\n"),
			  raw_throttle_min, raw_throttle_max,
			  motor_pwm_min, motor_pwm_max);
	motor_pwm_min = raw_throttle_max = 0;
	motor_pwm_max = raw_throttle_min = 4096;
}


/*
 * Local variables:
 *  compile-command: "make motor-commands.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
