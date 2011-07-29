/* blinky.c: LED blink program for PIC18F controllers. */
/*
 * This is the generic beginner/test program to blink a LED.
 */

/* Processor specific definitions, using the SDCC environment. */
#include "pic18fregs.h"

/* The GPIO pin that the LED is wired to. */
#ifndef LED_TRIS
#define LED_TRIS  TRISAbits.TRISA1
#endif
#ifndef LED_PIN
#define LED_PIN   PORTAbits.RA1
#endif


/* Sleazy software delay loop.  Unreliable, but avoids programming
 * a timer or using other hardware facilities.
 */
void delay_ms(long ms)
{
    long i;

    while (ms--)
        for (i=0; i < 330; i++)
            ;
}

void main(void)
{
    /* Set GPIO pin to output. */
    LED_TRIS = 0;

    /* Unending loop. */
    for (;;)
    {
        LED_PIN = 0;
        delay_ms(250);
        LED_PIN = 1;
        delay_ms(250);
    }
}

/*
 * Local variables:
 *  compile-command: "make blinky.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */

