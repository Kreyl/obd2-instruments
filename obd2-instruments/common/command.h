#ifndef _COMMAND_H
#define _COMMAND_H
/* command.h: Very small command line parser. */
/*
	$Id: $ $Date: 2010/23/01 00:00:21 $

	Written 2010 by Donald Becker and William Carlson
	This software may be used and distributed according to the terms
	of the GNU General Public License (GPL), incorporated herein by
	reference.  Firmware interacting with these functions are derivative
	works and thus are covered the GPL.  They must include an explicit
	GPL notice.
*/

/*
 * The table of variable names that may be interactively set.
 */
struct cmd_var_entry {
	PROGMEM char *name;					/* The variable name. */
	prog_uint16_t *ptr;			/* A pointer to the variable. */
	prog_uint16_t min, max;		/* The minimum and maximum values allowed. */
};
extern struct cmd_var_entry const cmd_var_table[];

/*
 * A similar table of functions that may be called.
 */
struct cmd_func_entry {
	PROGMEM char *name;		   /* The command name. */
	void (*funptr)(uint16_t val);
	prog_uint16_t min, max;	   /* The minimum and maximum values allowed. */
};
extern struct cmd_func_entry const cmd_func_table[];

void do_serial_port_char(unsigned char c);

/* Prototypes for serial.c functions. */
/* Get character from input FIFO, return -1 if empty. */
int uart_getch(void);
/* Put character to the output, return -1 if failed-full. */
unsigned char uart_putch(char c);
/* Configure the UART. */
void setup_uart(void);

extern int serprintf(const char *format, ...)
	__attribute__ ((format(printf, 1, 2)));

#define CMD_LINE_LEN	80		/* Maximum input line allowed. */

#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
