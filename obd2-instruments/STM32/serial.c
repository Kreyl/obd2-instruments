/* serial.c: UART queue support for small controllers.
 * Originally written for the Vvvvroom motor controller.
 */

#if ! defined(BAUD)
#define BAUD 9600
#endif

#if defined(STM32)
#include <armduino.h>
#if ! defined(F_CPU)
#define F_CPU 24*1000*1000		/* 24MHz */
#endif
#define cli()
#define sei()

#define CONCAT1(n,m) USART ## n ## _ ## m
#define UCONCAT2(n,m) CONCAT1(n,m)
#if defined(USART_NUM)
#define USARTNUM USART_NUM
#else
#define USARTNUM 2
#endif
/* Idiomatic way to get CPP to evaluate parmaters.  Yes, it is wonky. */
#define _CPP_CONCAT2b(a,b) a ## b
#define _CPP_CONCAT2a(num) _CPP_CONCAT2b(USART,num)
#define USART _CPP_CONCAT2a(USARTNUM)
#define USART_DR UCONCAT2(USARTNUM, DR)
#define USART_SR UCONCAT2(USARTNUM, SR)
#define USART_CR1 UCONCAT2(USARTNUM, CR1)
#define USART_CR2 UCONCAT2(USARTNUM, CR2)
#define USART_CR3 UCONCAT2(USARTNUM, CR3)
#define USART_BRR UCONCAT2(USARTNUM, BRR)
#define USART_Intr UCONCAT2(USARTNUM, Intr)
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
#define PARITY_NONE	0x000
#define PARITY_EVEN	0x1400		/* Forces 8bits+parity. */
#define PARITY_ODD	0x1600		/* Forces 8bits+parity. */

#define BITS_7_1	0x00		/* 7 bits must have parity */
#define BITS_7_2	0x2000
#define BITS_8_1	0x00
#define BITS_8_2	0x2000

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

/* USART interrupt.
 * The AVR has multiple specific interrupts.
 * The STM32 has a combined interrupt. */
IRQHandler(USART)
{
	unsigned status;
	unsigned char c;
	q_index i;

	status = USART_SR;
	if (status & USART_RXNE) {
		c = USART_DR;
		i = uart_rx.head + 1;
		if (i >= UART_RXBUF_SIZE)
			i = 0;
		if (i != uart_rx.tail) {		/* Check that the queue is not full. */
			uart_rx.buf[uart_rx.head] = c;
			uart_rx.head = i;
		}
		serial_rxbytes++;
	}
	if (status & USART_TXE) {
		i = uart_tx.tail;
		if (i != uart_tx.head) {
			USART_DR = uart_tx.buf[i++];
			if (i >= UART_TXBUF_SIZE) i = 0;
			uart_tx.tail = i;
		} else {
			/* Leave only the Rx interrupt enabled. */
			USART_CR1 &= ~USART_TXEIE;
		}
		serial_txbytes++;
	}
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
	USART_CR1 |= USART_TXEIE;
	return 0;
}

/* Configure the USART registers.
 * We set the UART to 9600,N,8,1 to match the Arduino, with the baud
 * rate set in the define above or overridden in the compile environment.
 * The pin mapping and direction have already been set up for us.
 */
void setup_uart(void)
{
	/* Re-initialize counts when called. */
	serial_txbytes = serial_rxbytes = 0;
	uart_rx.head = uart_tx.head = uart_rx.tail = uart_tx.tail = 0;

	/* Baud rate: Use the value from RM0041 table 122,
	 * UART1 uses PCLK2, all others use PCLK1
	 */
#if F_CPU / BAUD > 0xFFFF
#error "Baud rate out of range"
#endif
	USART_BRR = F_CPU / BAUD;

	USART_SR = 0;
	USART_CR2 = 0;
	USART_CR3 = 0x000;

	/* Enable the USART and receive interrupts.
	 * Nothing will happen unless interrupts are globally enabled.
	 */
	INTR_SETENA(USART_Intr);

	USART_CR1 = USART_UE | USART_TE | USART_RE | USART_RXNEIE;

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
