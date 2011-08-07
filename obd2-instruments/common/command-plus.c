/* command-plus.c: Serial command line interface for small controllers. */
/*
 * Generic command line interface for AVR programs.
 * The commands are of two types: set a variable, and call a function.
 *
 * The first is done with a table listing 16 bit unsigned variables.
 * The second is done with a similar table of function pointers.
 */
static const char versionA[] =
"command-plus: 1/14/2011 Copyright Donald Becker\n";

#if defined(STM32)
#include <armduino.h>
static int atoi(const char *nptr);
static int strcmp(const char *s1, const char *s2);

#elif defined (SDCC_pic16)
#include <stdint.h>
#include <string.h>
#define __attribute__(...) 
#define PGM_P const char *
#define PSTR(str) str
#define prog_uint16_t uint16_t

#elif defined(__AVR)

#if defined(__AVR_ATmega168__)
#define MEGA168
#endif
#if defined(__AVR_ATmega1280__)
#define MEGA1280
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(MEGA168)
#include <avr/iom168.h>
#else
#include <avr/iom1280.h>
#endif
#endif

#include "command.h"

/* A fixed command line buffer.  We have a miniscule memory, so this arbitrary
 * limit is a good thing. */
static uint8_t cmdpos;
static char cmd[CMD_LINE_LEN];

int atoi(const char *c)
{
	int res = 0;
	while (*c >= '0' && *c <= '9')
		res = res * 10 + *c++ - '0';
	return res;
}

#if defined(STM32)
static int strcmp(const char *s1, const char *s2)
{
	for(; *s1 == *s2; ++s1, ++s2)
        if (*s1 == 0)
            return 0;
    return *(unsigned char *)s1 < *(unsigned char *)s2 ? -1 : 1;
}

void *memcpy(void *dest, const void *src, long count)
{
	while (count--) {
		*(char*)dest++ = *(char*)src++;
	}
	return dest;
}
#endif

static void process_command(char *cmd, int val)
{
	uint8_t i;

	for (i = 0; cmd_var_table[i].name; i++) {
		if ( ! strcmp(cmd, cmd_var_table[i].name)) {
			if (val == -1)
				serprintf(PSTR("%s is %d.\n"), cmd, *cmd_var_table[i].ptr);
			else if (cmd_var_table[i].min <= val && val <= cmd_var_table[i].max)
				*cmd_var_table[i].ptr = val;
			else
				serprintf(PSTR("Value %d out of range for variable '%s'.\n"),
						  val, cmd);
			return;
		}
	}

	for (i = 0; cmd_func_table[i].name; i++) {
		if ( ! strcmp(cmd, cmd_func_table[i].name)) {
			(cmd_func_table[i].funptr)(val);
			return;
		}
	}
	serprintf(PSTR("Command not recognized: '%s'\n"), cmd);
	return;
}

/* Process a single character from the user input.
 * In the Cougar firmware this was mixed into the non-interrupt loop.
 * I reworked and simplified it.
 */
void do_serial_port_char(unsigned char c)
{

	/* If the character is not a CR we just echo it back and tack it onto the
	 * command line.  No support for line editing or even backspace. */
	if (c != 0x0d) {
		serprintf(PSTR("%c"), c);
		if (c == '\b') {
			if (cmdpos > 0)
				cmd[--cmdpos] = 0;
			else
				serprintf(PSTR(" "));
		} else if (cmdpos < (sizeof(cmd) - 1)) {
			cmd[cmdpos++] = c;
			cmd[cmdpos] = 0;
		}
		return;					/* Yup, a middle-of-function return. */
	}

	/* A CR: time to process the command line.
	 * Immediately echo back LF and CR, wait for prompt until command done.
	 */
	serprintf(PSTR("\n"));

	if (cmdpos > 0) {
		unsigned char i;
		int param = -1;			/* We default to -1 with no parameter. */

		for (i = 0; i < cmdpos; i++) {
			if (cmd[i] == ' ') {
				/* Terminate keyword at this position, parse numeric value.
				 * Note: we end up with a 0, not -1, for non-numeric text. */
				cmd[i] = 0; param = atoi(&cmd[i + 1]);
				break;
			}
		}
		cmdpos = i;				/* Command may parse args. */
		process_command(cmd, param);
	}
	/* Reset command string */
	cmdpos = 0; cmd[0] = 0;
	serprintf(PSTR("> "));
	return;
}


/*
 * Local variables:
 *  compile-command: "make command-plus.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
