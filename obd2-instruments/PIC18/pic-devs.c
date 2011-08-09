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

#include <stdint.h>
#include <pic18fregs.h>

#include "vvvvroom.h"
#include "can.h"
#include "command.h"


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
 */
void setup_io_ports(void)
{
	/* Select the external OSC1 8MHz crystal. */
	OSCCON = 0x74;
	/* Set outputs low before setting the direction. */
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	SSPCON1bits.SSPEN = 0;		/* Disable serial port */
	TRISA = 0xFF;				/* Analog inputs */
	TRISB = 0xFF;				/* Set all to inputs */
	TRISC = 0x8F;				/* Serial port (B7,B6). */
	/* Set the ADC to Vdd/Vss reference, AN0..AN5 as analog inputs. */
	ADCON1 = 0x0A;

	/* Initialize the USART */
	BAUDCON = 0;				/* Async 8 bit mode. */
	SPBRG = 0x19;
	TXSTA = 0xA4;
	RCSTA = 0x10;				/* Leave reset. */
	PIR2 = PIR1 = 0;
	PIE1 = 0x20;
	PIE2 = 0;
	/* Setup interrupts. */
	INTCON = 0xE0;				/* Enable intrs: peripherals and timer0 */
	RCONbits.IPEN = 1;			/* Set interrupt priority mode */
	IPR1bits.TXBIP = 1;			/* Set UART as a high priority interrupt. */
	INTCON2bits.TMR0IP = 0;		/* Set timer0 as a low priority interrupt. */
}

/*
 * With the PIC we use a timer only for system time.
 */
void setup_timers(void)
{
	T0CON = 5;
	/* Set timer period to 34285 = 0x85ED. */
	TMR0L = 0xED;
	TMR0H = 0x85;
}

/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
