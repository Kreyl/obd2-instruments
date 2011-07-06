/* serial.c: UART queue support for small controllers.
 */

#if ! defined(BAUD)
#define BAUD 9600
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
/* Using setbaud.h requires us to pre-define BAUD and F_CPU. */
#if ! defined(F_CPU)
#define F_CPU 16000000		/* 16MHz */
#endif
#include <util/setbaud.h>

#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(__AVR_ATmega168__)
#define MEGA168
#include <avr/iom168.h>
#elif defined(__AVR_ATmega1280__)
#define MEGA1280
#include <avr/iom1280.h>
#elif
#error No target AVR controller defined
#endif

/* Set the size of the receive and transmit buffers.
 * The size must fit within the queue index range.
 * The transmit buffer should be able to buffer a whole line, but can
 * smaller at the cost of busy-waiting.
 * The receive buffer only needs to handle a simple command.
 * We have little RAM on an ATMega, more is not better.
 */
#define UART_RXBUF_SIZE 16
#define UART_TXBUF_SIZE 128

/* Hardware constants for the serial port settings.
 * See setup_uart() below where they are used. */
#define PARITY_NONE	0x00
#define PARITY_EVEN	0x20
#define PARITY_ODD	0x30

#define BITS_7_1	0x04
#define BITS_7_2	0x0C
#define BITS_8_1	0x06
#define BITS_8_2	0x0E

#if defined(IOM8)
/* The IOM8 has only a single UART, thus the symbolic names omit the UART
 * index.
 */
#define SIG_USART0_RECV SIG_USART_RECV
#define SIG_USART0_DATA SIG_USART_DATA
#define UCSR0B UCSRB
#define UDR0 UDR
#define UBRR0L UBRRL
#define UBRR0H UBRRH
#elif defined(MEGA168)
#define SIG_UART_RECV SIG_USART_RECV
#define SIG_UART_DATA SIG_USART_DATA
#define UCSRB UCSR0B
#define RXEN RXEN0
#define TXEN TXEN0
#define RXCIE RXCIE0
#define UDRIE UDRIE0
#define UBRRL UBRR0L
#define UBRRH UBRR0H
#elif defined(_AVR_IOMXX0_1_H_)
/* It's silly to repeat the register bit definitions for each channel. */
#define RXCIE RXCIE0
#define UDRIE UDRIE0
#define RXEN RXEN0
#define TXEN TXEN0
#endif

typedef unsigned char q_index;

/* Queue state structure for a single direction, two per UART. */
struct uart_rx_fifo {
	volatile q_index head;
	volatile q_index tail;
	unsigned char buf[UART_RXBUF_SIZE];
} uart_rx;
struct uart_tx_fifo {
	volatile q_index head;
	volatile q_index tail;
	unsigned char buf[UART_TXBUF_SIZE];
} uart_tx;

/* Public statistics for the serial port - interrupt counts. */
volatile unsigned long serial_txbytes = 0;
volatile unsigned long serial_rxbytes = 0;

/* UART receive interrupt: With the AVR we can just assume that a character
 * has arrived. */
ISR(SIG_USART0_RECV)
{
	unsigned char c;
	q_index i;

	c = UDR0;
	i = uart_rx.head + 1;
	if (i >= UART_RXBUF_SIZE)
		i = 0;
	if (i != uart_rx.tail) {		/* Check that the queue is not full. */
		uart_rx.buf[uart_rx.head] = c;
		uart_rx.head = i;
	}
	serial_rxbytes++;
}

#if defined(ALT_USART)
/* Second/alternate UART receive interrupt.  We can accept input from two
 * serial ports.  This allows either the built-in USB connection and a
 * bluetooth module.  We assume it's not simultaneous and thus don't need to
 * lock.  A true race would be vanishingly rare with human input.
 */
ISR(SIG_USART1_RECV)
{
	unsigned char c;
	q_index i;

	c = UDR1;
	i = uart_rx.head + 1;
	if (i >= UART_RXBUF_SIZE)
		i = 0;
	if (i != uart_rx.tail) {		/* Check that the queue is not full. */
		uart_rx.buf[uart_rx.head] = c;
		uart_rx.head = i;
	}
	serial_rxbytes++;
}
#endif

/* UART Data Sent interrupt
 * Send the next character in the queue, or disable the Tx interrupt if
 * the queue has drained.
 */
ISR(SIG_USART0_DATA)
{
	q_index i;
	
	i = uart_tx.tail;
	if (i != uart_tx.head) {
		UDR0 = uart_tx.buf[i++];
		if (i >= UART_TXBUF_SIZE) i = 0;
		uart_tx.tail = i;
	} else {
		/* Leave only the Rx interrupt enabled. */
		UCSR0B = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	}
	serial_txbytes++;
}

/* Get the next character from the UART input FIFO.
 * Return -1 if the FIFO is empty.
 */
int uart_getchar(void)
{
	unsigned char c;
	q_index i, j;

	i = uart_rx.tail;
	j = uart_rx.head;			/* Must be atomic. */
	if (i != j) {
		c = uart_rx.buf[i++];
		if (i >= UART_RXBUF_SIZE) i = 0;
		uart_rx.tail = i;		/* Must be atomic. */
		return c;
	}
	return -1;
}

/*
 * Put character C on the UART transmit queue.
 * Return 0 on success, -1 if the queue is full.
 */
char uart_putchar(char c)
{
	q_index i, j;

	i = uart_tx.head + 1;
	if (i >= UART_TXBUF_SIZE) i = 0;
	j = uart_tx.tail;			/* Must be atomic. */
	if (i == j)					/* Queue full, report failure. */
		return -1;
	uart_tx.buf[uart_tx.head] = c;
	uart_tx.head = i;			/* Must be atomic. */
	/* Enable TX buffer empty interrupt. */
	UCSR0B = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE) | (1 << UDRIE);
	return 0;
}

/* Configure the USART registers.
 * We set the UART to 9600,N,8,1 to match the Arduino, with the baud
 * rate set in the define above or overridden in the compile environment.
 * Note that this does not yet support UART1..UART3
 */
void setup_uart(void)
{
	/* Re-initialize counts when called. */
	serial_txbytes = serial_rxbytes = 0;
	uart_rx.head = uart_tx.head = uart_rx.tail = uart_tx.tail = 0;

	/* Use the vaules calculated in setbaud.h */
	UBRR0L = UBRRL_VALUE;
	UBRR0H = UBRRH_VALUE;
#if defined(IOM8)
	/* The mega8 overlaps UBRRH with the C control register, using
	 * the high bit, URSEL, to select which is written.
	 * We always use 8N1 to communicate.
	 */
	UCSRC = PARITY_NONE | BITS_8_1 | (1 << URSEL);
#else
	UCSR0C = PARITY_NONE | BITS_8_1;
#endif
	/* Enable the USART and receive interrupts.
	 * Nothing will happen unless interrupts are globally enabled.
	 */
	UCSR0B = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);

#if defined(ALT_USART)
	UBRR1L = UBRRL_VALUE;
	UBRR1H = UBRRH_VALUE;
	UCSR1C = PARITY_NONE | BITS_8_1;
	UCSR1B = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	{ volatile char foo = UDR1;}
#endif

	return;
}

/*
 * Local variables:
 *  compile-command: "make serial.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
