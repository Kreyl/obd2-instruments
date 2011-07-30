/* pci-devs.c: PIC18 device specific routines VVVVroom instrument system. */
/*
  Written 2010-2011 by Donald Becker and William Carlson.

  These are the hardware interface routines for PIC18 version of the
  VVVVroom EV instrument network.  These are the functions that are most
  intimately tied to the specific device's registers and bits.

	This file contains the following subsystems
	Configuration data read and write, using flash instead of EEPROM
	Processor initialization, including clock configuration
	I/O port configuration
	A/D and PWM-timer configuration
	SPI initialization.

	Some hardware-specific details remain in the other subsystem code
	for convenience
	Updating the PWM percentage
	Reading the A/D converter values
	SPI reads and write
*/
static const char versionA[] =
"$Id: pic-devs.c $\n";

/* Watchdog section.  Ready for future functionality. */
void wdt_reset(void)
{
}
void watchdog_disable(void)
{
	/* The watchdog is fuse enabled and cannot be disabled on the PIC18. */
}
void watchdog_enable(void)
{
}

/* Configure the I/O port functions and directions.
 * This changes with the hardware platform and specific MCU part.
 * We need to set up the PWM timer, clock timer, ADC, LED pins,
 * SPI, USART and CAN.
void setup_io_ports(void)
{
}

/*
 * With the PIC we use a timer only for system time.
 */
void setup_timers(void)
{
}

/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
