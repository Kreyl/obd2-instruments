/* can-bus.c: OBD2 CAN interface for the VVVVroom motor controller. */
/*
 * CAN bus interface and reporting.
 * This file has the code for responding to CAN bus messages, reporting
 * state, status, statistics and setting the configuration.
 *
 * It combines the OBD2 protocol code with the driver for a SPI connected
 * MCP2515 into a single source file.  This is an awkward mix of layers,
 * but makes it easier to share with the AVR-based Cougar motor controller.
 *
 * This code currently implements most relevant OBD2 PIDs, with a structure
 * that makes it easy to add more and to report additional values.
 * We respond to SAE J1979 PIDs as well as our own "manufacture specific"
 * PIDs.
 *
 * We use an 8MHz clock from either an AVR timer output or a local crystal.
 * This is high enough to support the common 250K and 500K NBR
 * (Nominal Bit Rate) CAN bus speeds.
 * The controller needs at least 10 cycles per bit, preferably 16, to sample
 * and resynchronize, so supporting a future 1M NBR requires a local crystal.
 *
 * We always configure to 500Kbps and start broadcasting, expecting that
 * other devices will be silently waiting to autobaud against our traffic.
 *
 * References for understanding this code
 *  MCP2515 datasheet rev D, DS21801D (default for page number references)
 *  http://en.wikipedia.org/wiki/On-board_diagnostics
 *  http://en.wikipedia.org/wiki/OBD-II_PIDs
 *  http://en.wikipedia.org/wiki/ISO_15765-2
 *  http://www.semiconductors.bosch.de/pdf/can2spec.pdf
 *  SAE-J2190
 *
 * Notes:
 * DLC (data length) is always 8 with automotive CAN
 *
 * Addresses are poorly considered in CAN.  They are neither source
 * nor destination addresses.  They are abitrarily assigned and not
 * really discoverable.  Pretty much everything dynamic has to be
 * a broadcast, and no 'general purpose CAN stack' is feasible.
 *
 * OBD specifies a 'funcitonal ID' of 0x7DF that acts as a broadcast
 * address, and physical addresses of 0x7E0-7EF.  0x7E0-0x7E7 are for
 * for tester-to-ECU communication, and 0x7E8-0x7EF are for ECU-to-tester.
 * A diag tool will broadcast on 0x7DF, and use the ID from the response
 * for subsequent frames with multi-frame messages.  We respond from 0x7E8.
 *
 * Internal physical addresses are mostly arbitrary, and in-network
 * communication uses the already-known ID.  We use 0x420, and respond to
 * the same set of queries as with 0x7DF/0x7E8.
 *
 * There must always be another active CAN device on the bus that provides
 * an ACK response to successfully transmit.
 * If using an ELM327 v1.42b and later as the only other bus
 * device, turn silent monitoring off with "AT CSM1".  Otherwise it will
 * only provide the ACK to messages it is listening to.
 */
static const char versionA[] =
"can-bus.c: $Id: can-bus.c 223 2011-05-13 15:40:59Z becker $ 1/30/2011 Copyright Donald Becker\n";

#if defined(STM32)
/* The STM32 doesn't need the awkward Harvard architecture hacks of the AVR. */
#include <armduino.h>
#define PGM_P void *
#define PSTR(str) str
#define PROGMEM const
#define pgm_read_byte(addr) (*addr)
extern char *strcpy_P(char *dest, const char *src);

#elif defined(__AVR_ATmega168__)
#define MEGA168
#elif defined(__AVR_ATmega1280__)
#define MEGA1280
#endif

/* We need very few library routines, mostly AVR-specific hardware
 * macros. */
#if defined(STM32)
extern void *memset(void *s, int c, long n);

#else
#include <stdlib.h>
#include <string.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(MEGA168)
#include <avr/iom168.h>
#elif defined(MEGA1280)
#include <avr/iom1280.h>
#else
#error "Undefined processor type."
#endif

#if defined(COUGAR)
/* Tweaks to work with the Cougar firmware. */
#include "cougar-export.h"
#include "cougar.h"
#ifndef tach_get_QRPM
#define tach_get_QRPM() 1234*4
#endif

#define use_direct_pwm 0
#endif

/* We need to deduce our own clock from counter_1k. */
uint16_t local_clock, tm_seconds;
#endif

#include "vvvvroom.h"
#include "can.h"

extern volatile uint16_t pwm_width;	/* EV controller "throttle plate" 0..511  */
static inline uint8_t engine_load(void)
{
	return pwm_width >> 1;
}

/* The default VIN, used if not set from the EEPROM config.
 * A VIN is typically only 17 ASCII characters. */
char VIN_str[20] = "WBAAA1305H2320124";
/* Set to run at fixed speed.  Used for cruise control or to run A/C, PS pump
 * etc. */
uint16_t PTO_fast_idle;			/* Commanded motor speed in QRPM. */
/* A single global structure for constructing commands makes it easier to
 * add new responses.
 */
struct CAN_command can_cmd;

/* Try to put a little abstraction on the bit banging and very
 * different hardare.
 */
#if defined(STM32)
#define PB_CAN_CS 12		/* Abitrary digital pin used for active-low /CS */
static inline void CAN_CS_ENABLE(void)
{
	GPIOB_BRR = (1 << PB_CAN_CS);		/* Chip select low */
}
static inline void CAN_CS_DISABLE(void)
{
	GPIOB_BSRR = (1 << PB_CAN_CS);		/* Chip select high */
}
#define SPI_CLK_DIV SPI_BRdiv4
#define SPDR	SPI1_DR
#define SPSR	SPI1_SR
#define SPI_CR1 SPI1_CR1
#define SPI_CR2 SPI1_CR2
#else
/* GCC-avr compiles these macros to single instructions. */
#define PB_CAN_CS DDB0		/* Abitrary digital pin used for active-low /CS */
#define CAN_CS_ENABLE	PORTB &= ~(1 << PB_CAN_CS);		/* Chip select low */
#define CAN_CS_DISABLE	PORTB |= (1 << PB_CAN_CS);		/* Chip select high */
#define SPI_CLK_DIV	(0 << SPR0) /* 0,1,2,3 is divisor by 4,16,64,128  */
#define SPI_RXNE  (1<<SPIF)
#endif

/* Our CAN ID for operational (non-diagnostic, higher priority) frames. */
#define CAN_PHY_ADDR 0x420

/* Symbolic constants for the MCP2515 CAN controller.
 * I omit most single-use symbolic names: they don't add clarity, and
 * introduce an opportunity for errors when writing and verifying the code.*/

/* These SPI commands are mostly MCP2515-specific. */
enum MCP2515_Commands {
	MCP_Write=0x02,	MCP_Read=0x03, MCP_BitMod=0x05,
	MCP_LoadTx=0x40, MCP_LoadTx0=0x40, MCP_LoadTx1=0x42, MCP_LoadTx2=0x44,
	MCP_RTS_Tx=0x80, MCP_RTS_Tx0=0x81, MCP_RTS_Tx1=0x82, MCP_RTS_Tx2=0x84,
	MCP_ReadRx0=0x90, MCP_ReadRx0Payload=0x92,
	MCP_ReadRx1=0x94, MCP_ReadRx1Payload=0x96,
	MCP_ReadStatus=0xA0, MCP_RxStatus=0xB0, MCP_Reset=0xC0,
};

/* CANINTF Register bits. */
enum MCP2515_CANIntf_bits {
	MCP_Rx0IF=0x01, MCP_Rx1IF=0x02, MCP_Tx0IF=0x04, MCP_Tx1IF=0x08,
	MCP_Tx2IF=0x10, MCP_ErrIF=0x20, MCP_WakIF=0x40, MCP_MErrF=0x80,
};

#define CAN_LOOPBACK 0x40		/* Set to 0x40 to test with loopback mode */

/* Internal state and structures. */
uint8_t CAN_reset_done = 0;
uint8_t CAN_enabled = 0;
uint8_t can_verbose = 0;

uint16_t local_clock, tm_seconds;

/* Multi-frame (ISO-TP 15765-4) response state.
 * The VIN request was the first multi-line response I wrote, so
 * the state continues to use that name.
 */
struct CAN_multiframe {
	int16_t msg_cnt;		 /* If non-zero, more remains to be sent. */
	char *message;			 /* Current message pointer. */
	uint8_t frame_idx;		 /* Frame index for header. */
	uint8_t delay, window;			/* Info from flow control. */
	uint16_t last_send;				/* Use for flow control delay. */
	uint16_t id;					/* Usually the ECU ID. */
};
static struct CAN_multiframe VIN = {msg_cnt:0, };

/* State of the Tx and Rx buffers.
 * It takes five extra bytes, 80 clock cycles, to re-write the Tx header.
 * We record the previous value to check if its unchanged.
 */
struct tx_buf_state {
	uint8_t  TxCtrl;				/* TxBnCTRL contents. */
	uint16_t TxID;					/* TxBnSIDL address. */
} txbuf[3];

/* Not used -- better known as the numbers than the name.
 * Here for reference only. */
enum CAN_modes {
	ShowCurrentData=1, ShowFreezeData=2,
	ShowStoredDiagCodes=3, ClearDiagCodes=4,
	TestResultsO2=5, TestResults=6,
	ShowDiagErrors=7, CANControl=8, VehicleInfo=9, PermanentDTC=10,
};

/* Out tenative "manufacture-specific" commands. */
enum CAN_Commands {
	CAN_PI_PARAM, CAN_THROTTLE_RANGE,
};

#if defined(COUGAR_SETVAR)
/* A list of the operating variables that may be read and set. */
struct canned_response {
	unsigned int *var1, *var2;
	unsigned char flags;
	unsigned char pad;
} can_msg_list[] = {
	{(unsigned *)&config.Kp, (unsigned *)&config.Ki, 0},	/* CAN_PI_PARAM: */
	{&config.throttle_min_raw_counts, &config.throttle_min_raw_counts, 0},
	{&config.throttle_fault_raw_counts, 0, 0},
	{&config.throttle_pos_gain, &config.throttle_pwm_gain, 0},
	{(unsigned *)&config.current_ramp_rate, 0, 0},
	{&config.pwm_filter, 0, 0},
	{&config.motor_os_th, &config.motor_os_ft, 0},
	{&config.motor_os_dt, &config.pwm_deadzone, 0},
	{&config.motor_sc_amps, 0, 0},
	{&config.battery_amps_limit, 0, 0},
	{&config.precharge_time, 0 /* Precharge percent */, 0},
};
#endif

/* Named constants for the configuration registers.
 * These are only used once, but are a bit involved.
 * We set the prescaler to '0', which means divides by 2.
 * The sync segment is fixed at 1
 * The propagation segment is 0, which means 1 TQ
 * The phase 1 segment is 2, which means 3TQ (minimum 1TQ)
 * The phase 2 segment is 2, which means 3TQ (minimum 2TQ)
 */
#define MCP_CNF1  0x00 		/* Prescaler at minimum, 4MHz TQ */
#define MCP_CNF2  0x90		/* BTLMODE, PHSEG=2, PRSEG=0 */
#define MCP_CNF3  0x02		/* PHSEG2=2, no wake-up, CLKOUT pin is clk */

#if 0
/*
 * This will become the manufacturer-specific section.
 * We will respond with our state variables using the table above.
 */
void can_state(void)
{
	int var1, var2;
	unsigned char var3;
	switch (msg_type) {
	case CAN_PI_PARAM:
		var1 = config.Kp;
		var2 = config.Ki;
		break;
	case CAN_THROTTLE_RANGE:
		var1 = config.throttle_min_raw_counts;
		var2 = config.throttle_max_raw_counts;
		break;
	default:
		break;
	}
}

#endif

#if 0
/* Temperature normalizaiton calibration using a linear interpolation table.
 * The table is build using calibration numbers taken at a few temperatures,
 * including an estimate for the extreme lower and upper raw values.
 */
struct {int raw, C; } temp_table[] = {
	{0, -50},					/* Fake a lower bound: raw_adc of 0 is -50C */
	{400, 0},					/* Measured value of ice at 0C */
	{500, 20},					/* Comfy room at 20C */
	{700, 100},					/* Measured value of boiling water at 100C */
	{1024, 200},				/* Estimate upper bound: raw_adc of 1024 200C */
};
int raw_to_calibrated(int adc_raw)
{
	int i;
	int celsius;

	/* Scan the table, knowing that we can't fall off the end. */
	for (i = 1; adc_raw < temp_table[i].raw; i++)
		;
	/* i is now the index for the higher value. Interpolate.  */
	celsius = (adc_raw - temp_table[i-1].raw)*temp_table[i].C +
		(temp_table[i-1].raw - adc_raw)*temp_table[i-1].C;
	celsius /= temp_table[i].raw - temp_table[i-1].raw;
	return celsius;
}
#endif

/* Sending a byte on SPI is easy: write, check status for completion,
 * and read the byte simultaneously clocked in.
 */
char inline SPI_Transmit(char data)
{
	SPDR = data;
	while ( ! (SPSR & SPI_RXNE))
		;
	return SPDR;
}

/* Send a sequence of SPI bytes from constant/program space.
 * Size must be 1..255.  Using 0 will send 256 bytes.
 * Using this makes the code a bit harder to read, but really shrinks
 * the generated code size.
 */
void SPI_transmit_array(const prog_uint8_t *data, uint8_t size)
{
	CAN_CS_ENABLE;
	do {
		SPI_Transmit(pgm_read_byte(data++));
	} while (--size != 0);
	CAN_CS_DISABLE;
}

/* Initialize the SPI interface.
 * For the STM32 processors
 *			SS   SCK  MOSI MISO
 *  SPI1	PA4  PA5  PA7  PA6
 *  SPI1-A  PA15 PB3  PB5  PB4
 *  SPI2	PB12 PB13 PB15 PB14
 *  SPI2-A
 * For the AVR processors e.g. Mega 1280
 *  					SS SCK MOSI MISO
 *  AT90USB82/162 		PB0	PB1 PB2 PB3
 *  ATmega48/88/168/328 PB2	PB5 PB3 PB4
 *  ATtiny8			 	-	- 	PA6	PA5
 * We set these SPI pins on port B as outputs:  MOSI SCK SS
 * And, grrrr, a zillion pointless defines e.g. 'DDB2' and not a single one
 * to make chip-independent SPI code possible e.g. for MOSI/MISO/SCK/SS.
 */

uint8_t CAN_dev_init(void)
{
	uint8_t i;

	CAN_CS_DISABLE;

#if defined(STM32)
	/* Set the GPIO pins to alternate function bits to enable SPI outputs. */
	/* PB13/15 alternate function output (fast), PB12 AF output, PB14 input */
	GPIOB_CRH |= 0xB4B30000;
	/* Turn on: SPI enable, 8 bit, MSB first, 0,0, Master, F_OSC/4. */
	SPI_CR1 = 0x00 | SPI_SPE | SPI_CLK_DIV;
	SPI_CR2 = 0;
#else
	DDRB |= ((1<<DDB2)|(1<<DDB1)|(1<<PB_CAN_CS));
	/* Turn on: SPI enable, MSB first, 0,0, Master, F_OSC/4. */
	SPCR = (1 << SPE) | (1 << MSTR) | SPI_CLK_DIV;
	SPSR = 1 << SPI2X;				/* Double clock to 8MHz */
#endif

	/* Send a reset as a stand-alone command and monitor completion. */
	/* This is SPI_Transmit(MCP_Reset) with a fail check. It typically
	 * takes 3-4 loops to complete. */
	CAN_CS_ENABLE;
	SPDR = MCP_Reset;
	for (i = 255; i > 0; i--) {
		if (SPSR & SPI_RXNE)
			break;
	}
	CAN_CS_DISABLE;
	SPDR;			/* Clear data register from init. */
	CAN_reset_done = 1;
	memset(txbuf, 0, sizeof txbuf);
	return i;
}

uint8_t can_get_status(void)
{
	uint8_t status;
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_ReadStatus);
	status = SPI_Transmit(0xFF);		/* 0xFF: detect non-response. */
	CAN_CS_DISABLE;
	return status;
}

/* Debugging code. */
void CAN_show_registers(int8_t base)
{
	unsigned char i;
	unsigned char result[16];

	CAN_CS_ENABLE;
	SPI_Transmit(base < 0 ? MCP_RxStatus : MCP_Read);  /* MCP_Read */
	SPI_Transmit(base);
	for (i = 0; i < 16; i++)
		result[i] = SPI_Transmit(0x00);
	CAN_CS_DISABLE;

	strcpy_P(uart_str, PSTR(" CAN registers 0xNN:"));
	u16x_to_str(&uart_str[17], base, 2);
	uart_putstr();
	strcpy_P(uart_str, PSTR(" XX"));
	for (i = 0; i < 16; i++) {
		u16x_to_str(&uart_str[1], result[i], 2);
		uart_putstr();
	}
	uart_str[0] = '\r';
	uart_str[1] = '\n';
	uart_str[2] = '\0';
	uart_putstr();
	return;
}

/* Controller register initialization.
 * Uglier than the earlier version, but doing it this way shrinks the
 * size from 0x242 to 0x1xx bytes.
 */
const prog_uint8_t mcp_reg00_init[] = {
	MCP_Write, 0x00,					/* Write Rx filter 0 */
	/* Accept OBD functional diag ID 0x7DF, with a zero EID. */
	0x7DF >> 3, (0x7DF << 5)&0xFF, 0x00, 0x00,
	/* Our physical address in #1 */
	CAN_PHY_ADDR >> 3, (CAN_PHY_ADDR << 5)&0xFF, 0x00, 0x00,
   /* Accept OBD physical IDs 0x7E0-0x7EF */
	0x7E0 >> 3,	(0x7E0 << 5)&0xFF, 0x00, 0x00,
	/* Continue on to write the output pin config registers. */
	0x00,								/* BFPCTRL: pin control pg 29 */
	0x00,								/* TXRTSCTRL: pin control pg 19 */
	0x00, 0x80,							/* CANCTRL, leave us in config mode. */
	0,0,0,0, 0,0,0,0, 0,0,0,0			/* Clear ID match 3, 4 and 5. */
};

const uint8_t PROGMEM mcp_reg20_init[] = {
	MCP_Write, 0x20,					/* Mask for filter 0 and 1 */
	/* ID match mask.  Clear EID mask to avoid filtering on payload. */
	0x7FF >> 3, (0x7FF << 5)&0xFF, 0x00, 0x00,
	/* Mask for filter 2,3 and 4 */
	0x7F0 >> 3, (0x7F0 << 5)&0xFF, 0x00, 0x00,
	/* Continue writing to set the configuration, we are now at CNF3, pg42. */
	MCP_CNF3, MCP_CNF2, MCP_CNF1,
	0x00, 0x00, 0x00, 0x00,			   /* CANINTE, CANINTF, EFLG, CANSTAT */
	0x07,							   /* CANCTRL: normal mode, /8 clock out */
};

/* RXB0CTL pg 27 Only 11 bit IDs, roll over bufs */
const uint8_t PROGMEM mcp_reg60_init[] = {
	MCP_Write, 0x60, 0x24,
};

uint8_t can_setup(void)
{
	uint8_t i, status;

	CAN_enabled = 0;

	/* This will be cleaned up after we verify that Cougar works. */
	if (CAN_reset_done == 0 && CAN_dev_init() == 0) {
		strcpy_P(uart_str, PSTR("CAN reset failed! 0x--\r\n"));
		u16x_to_str(&uart_str[20], SPSR, 2);
		uart_putstr();
		return 1;
	}

	/* Wait until a read status command works.
	 * This typically takes 4 loops after a reset, so allow 10x.*/
	for (i = 0; i < 40; i++) {
		status = can_get_status();
		if (can_get_status() != 0xFF) {
			CAN_enabled = 1;
			break;
		}
	}

	if (status == 0xFF) {
		strcpy_P(uart_str, PSTR("MCP2515 reset failed.\r\n"));
		uart_putstr();
	}

#if 1
	SPI_transmit_array(mcp_reg00_init, sizeof mcp_reg00_init);
	SPI_transmit_array(mcp_reg60_init, sizeof mcp_reg60_init);
	SPI_transmit_array(mcp_reg20_init, sizeof mcp_reg20_init);
#else
	/* Write the Rx filter patterns with our likely addresses. pg 34 */
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x00);					/* Writing Rx filter 0 */
	SPI_Transmit((0x7DF >> 3)&0xFF);	/* Accept OBD standard 0x7DF */
	SPI_Transmit((0x7DF << 5)&0xFF);
	SPI_Transmit(0x00);					/* Needlessly clear the EID */
	SPI_Transmit(0x00);
	SPI_Transmit(CAN_PHY_ADDR >> 3);	/* Our physical address in #1 */
	SPI_Transmit((CAN_PHY_ADDR << 5)&0xFF);
	SPI_Transmit(0x00);
	SPI_Transmit(0x00);
	SPI_Transmit(0x7E0 >> 3);			/* Accept OBD alternate 0x7E0 */
	SPI_Transmit((0x7E0 << 5)&0xFF);
	SPI_Transmit(0x00);
	SPI_Transmit(0x00);
	/* Continue on to write the output pin config registers. */
	SPI_Transmit(0x00);					/* BFPCTRL: pin control pg 29 */
	SPI_Transmit(0x00);					/* TXRTSCTRL: pin control pg 19 */
	CAN_CS_DISABLE;

	/* Not really needed. Write RxB1CTL if really needed.  */
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x60);					/* RXB0CTL pg 27 */
	SPI_Transmit(0x24);					/* Only 11 bit IDs, roll over bufs */
	CAN_CS_DISABLE;

	/* Set the Rx masks for an 11 bit identifier. pg 38 */
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x20);					/* Mask for filter 0 and 1 */
	SPI_Transmit(0x7FF >> 3);
	SPI_Transmit(0x7FF << 5);
	SPI_Transmit(0x00);					/* Clear EID to avoid filtering */
	SPI_Transmit(0x00);					/* on payload bytes. */
	SPI_Transmit(0x7F0 >> 3);			/* Mask for filter 2,3 and 4 */
	SPI_Transmit((0x7F0 << 5)&0xFF);	/* Second verse same as the first */
	SPI_Transmit(0x00);
	SPI_Transmit(0x00);
	/* Just continue writing to set the configuration, we are now at CNF3. */
	SPI_Transmit(MCP_CNF3);				/* CNF3 pg42 */
	SPI_Transmit(MCP_CNF2);				/* CNF2 */
	SPI_Transmit(MCP_CNF1);				/* CNF1 */
	SPI_Transmit(0x00);					/* CANINTE */
	SPI_Transmit(0x00);					/* CANINTF */
	SPI_Transmit(0x00);					/* EFLG */
	SPI_Transmit(0x00);					/* CANSTAT */
	SPI_Transmit(0x07);			/* CANCTRL: normal mode, /8 clock out */
	CAN_CS_DISABLE;
#endif

	return 0;
}

/* Read the state of TXnRTS input pins.  Report the raw value.
 * Tx0RTS  ---- V---
 * Tx1RTS  ---V ----
 * Tx2RTS  --V- ----
 * We do not shift as the caller will be doing a bit mask anyway.
 * If you want clean abstractions, don't use an 8 bit processor.
 */
uint8_t mcp2515_get_pins(void)
{
	uint8_t status;
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Read);
	SPI_Transmit(0x0D);					/* TXRTSCTRL */
	status = SPI_Transmit(0x00);
	CAN_CS_DISABLE;
	return status;
}

/* Set the RXnBF output pins to the desired state.
 * The value is directly written: 00xy 1100
 * Pass 0x1C to set pin Rx0BF high.
 * Pass 0x2C to set pin Rx1BF high.*/
void mcp2515_set_pins(uint8_t pinval)
{
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x0C);					/* BFPCTRL: pin control pg 29 */
	SPI_Transmit(pinval);
	CAN_CS_DISABLE;
	return;
}

/* Transmit the frame in can_cmd using buffer BUF_NUM.
 * At minimum we must write TxBnSIDH TxBnSIDL TxBnDLC and the
 * message itself.
 *
 * We are extra clever and scan for an empty Tx buffer with a matching
 * previuosly-written header.  This is probably foolish, as it could
 * result in out-of-order multi-frame responses, and priority inversion.
 *
 * The original, trivial version with fixed ID assignment is probably better.
 */
int8_t can_transmit(void)
{
	uint8_t i, skip_header, buf_num;
	uint16_t SID = 0x7E8;		/* Eventually change to dynamic address.  */

#if ! defined(SIMPLE_TX_BUF)
	buf_num = 3;
	skip_header = 0;
	for (i = 0; i < 3; i++) {
		if ((txbuf[i].TxCtrl & 0x08) == 0) {
			buf_num = i;
			if (txbuf[buf_num].TxID == SID) {
				skip_header = 1;
				break;
			}
		}
	}
	if (buf_num >= 3)
		return 1;
#else
	if ((txbuf[0].TxCtrl & 0x08) == 0) {
		buf_num = 0;
	} else if ((txbuf[1].TxCtrl & 0x08) == 0) {
		buf_num = 1;
	} else
		return 1;
	skip_header = (txbuf[buf_num].TxID == SID);
#endif

	/* We have a special command to start writing at 0x31/41/51. */
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_LoadTx0 + (buf_num<<1) + skip_header);
	if ( ! skip_header) {
		/* Load fresh values into the Tx buffer. */
		SPI_Transmit(SID >> 3);			/* TXBnSIDH */
		SPI_Transmit(SID << 5);			/* TXBnSIDL: No extended ID */
		SPI_Transmit(0x00);				/* TXBnE8 */
		SPI_Transmit(0x00);				/* TXBnE0 */
		SPI_Transmit(0x08);				/* TXBnDLC we always send 8 bytes */
		txbuf[buf_num].TxID = SID;
	}
	for (i = 0; i < 8; i++)
		SPI_Transmit(((unsigned char *)&can_cmd)[i]);
	CAN_CS_DISABLE;
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_RTS_Tx + (1<<buf_num));
	txbuf[buf_num].TxCtrl |= 0x08;
	CAN_CS_DISABLE;
	if (can_verbose > 3) {
		strcpy_P(uart_str, PSTR(" CAN Tx 0xHHH HH HH HH HH HH HH HH.\n\r"));
		u16x_to_str(&uart_str[10], SID, 3);
		u16x_to_str(&uart_str[14], can_cmd.cnt, 2);
		u16x_to_str(&uart_str[17], can_cmd.mode, 2);
		u16x_to_str(&uart_str[20], can_cmd.pid, 2);
		u16x_to_str(&uart_str[23], can_cmd.dataA, 2);
		u16x_to_str(&uart_str[26], can_cmd.dataB, 2);
		u16x_to_str(&uart_str[29], can_cmd.dataC, 2);
		u16x_to_str(&uart_str[32], can_cmd.dataD, 2);
		uart_putstr();
	}
	return 0;
}

#if defined(CAN_SLEEPMODE)
/* CAN controller interrupt.
 *
 * This usually means that we have an inbound message while in sleep mode.
 * No action required except to wake.
 */
ISR(INT4_vect)
{
	return;
}
void mcp2515_sleep(uint16_t val)
{
	CAN_reset_done = 0;

	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x2B);					/* CANINTE */
	SPI_Transmit(MCP_WakIF);			/* Allow wake interrupt */
	SPI_Transmit(0x00);					/* Clear interrupt sources */
	CAN_CS_DISABLE;

	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x0F);					/* CANCTRL */
	SPI_Transmit(0x20);					/* Set sleep mode. */
	CAN_CS_DISABLE;

	return;
}
#endif

/* This is the initially interesting part: respond to standard OBD2
 * messages.  To keep the code simple, the surrounding code deals with
 * addressing and message format.
 */
uint8_t OBD2_respond(void)
{
	uint8_t count = 0;

	switch(can_cmd.mode) {
	case 1: switch(can_cmd.pid) {		/* ShowCurrentData */
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0xB8;	  	/* 01 -- 03 04 05 -- -- -- */
			can_cmd.dataB = 0x13;		/* -- -- -- 0c -- -- 0f 10 */
			can_cmd.dataC = 0x80; 		/* 11 -- -- -- -- -- -- -- */
			can_cmd.dataD = 0x13;		/* -- -- -- 1c -- -- 1f 20 */
			count = 6;
			break;
		case 0x01:
			can_cmd.dataA = fault_bits ? 0x81 : 0;
			can_cmd.dataB = 0x00;	/* Nothing to see here.  Move along now. */
			can_cmd.dataC = 0x00;	/* No tests available. */
			can_cmd.dataD = 0x00;
			count = 6;
			break;
		case 0x03:
			/* Report closed loop when the current sensor can be read. */
			can_cmd.dataA = use_direct_pwm ? 8:2;
			can_cmd.dataB = 0;	/* with no second fuel system */
			count = 4;
			break;
		case 0x04:	/* Calculated Engine load 0..255. */
			/* Percentage of available torque, not rated torque.
			 * On the QAR controller we report PWM percentage, although
			 * amps is the best estimate of torque. */
			can_cmd.dataA = engine_load();
			count = 3;
			break;
		case 0x05:	/* Coolant temp A-40 -40C..+215C, we use heatsink temp. */
#if 0
			/* Assume a LM335 where  Kelvin = 100 * 5V * (count / 1024)
			 * Our baseline temp is -40C or 233.15 Kelvin, or 2.33V
			 * which is an A/D reading of about 477
			 * DataA = ((500 * rt_data.raw_hs_temp)/ 1024) - 233  */
			can_cmd.dataA = (125 * (rt_data.raw_hs_temp - 477)) >> 8;
#else		/* We are using using a MCP9701 sensor on this channel. */
			/* MCP9701 temp sensor conversion. */
			can_cmd.dataA = (rt_data.raw_hs_temp + 78)>>2;
#endif
			count = 3;
			break;
		case 0x0C:		/* Engine RPM, revs per 4 minutes.  */
			{
				uint16_t qrpm = tach_get_QRPM();
				can_cmd.dataA = qrpm >> 8;
				can_cmd.dataB = qrpm;
				count = 4;
			}
			break;
		case 0x0F:	/* Intake temp A-40, -40C..+215C, use second temp sensor. */
			/* The calculation is for a MCP9701 sensor.
			 * It outputs 400mV at 0C, and increases by 19.5mV/C. */
			if (raw_adc[3] < 1000) {			/* No report if missing. */
#if 0											/* MCP9701 sensor */
				can_cmd.dataA = (raw_adc[3]+78)>>2;
#else				/* Ooops, use LM335 conversion instead. */
				can_cmd.dataA = (125 * (raw_adc[3] - 477)) >> 8;
#endif
				count = 3;
			}
			break;
		case 0x10: {
			/* Nominally MAF (air flow) in 0.01 gram/sec.  We instead report
			 * loosely-calibrated amps, largely because it is a standard
			 * ScanGauge meter. */
			uint16_t amps = current_fb * 100;
			can_cmd.dataA = amps >> 8;
			can_cmd.dataB = amps;
			count = 4;
			break;
		}
		case 0x11:	/* Absolute throttle pos. 0..255, we use normalized
					 * pedal position. */
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			count = 3;
			break;
		case 0x1C:
			can_cmd.dataA = 9;	/* We respond to EOBD, OBD and OBD II */
			count = 3;
			break;
		case 0x1E:
			can_cmd.dataA = PTO_fast_idle ? 1 : 0;
			count = 2;
			break;
		case 0x1F: {		/* Run time since start.  */
			can_cmd.dataA = tm_seconds >> 8;
			can_cmd.dataB = tm_seconds;
			count = 4;
			break;
		}
		case 0x20:				/* PID 21-40 support, hand calculated. */
			can_cmd.dataA = 0x00;
			can_cmd.dataB = 0x00;
			can_cmd.dataC = 0x00;
			can_cmd.dataD = 0x01; 		/* Only 0x40 */
			count = 6;
			break;
		case 0x40:				/* PID 41-60 support, hand calculated. */
			can_cmd.dataA = 0x0A;		/* -- -- -- -- 45 -- 47 -- */
			can_cmd.dataB = 0x00;		/* None */
			can_cmd.dataC = 0x80;		/* 51 -- -- --  -- -- -- -- */
			can_cmd.dataD = 0x10; 		/* 59: -- -- -- 5C  -- -- -- --   */
			count = 6;
			break;
#if 0
		case 0x42: {
			/* Control module voltage: 12V expected, we report 5V supply. */
			uint16_t voltage;
			/* Measure the 1.1V ref, round a bit to allow exactly 5.000V */
			voltage = 1125000L / rt_data.raw_1_1V;
			can_cmd.dataA = voltage >> 8;
			can_cmd.dataB = voltage;
			break;
		}
#endif
		case 0x45:				/* Relative throttle position. */
		case 0x47:				/* Absolute throttle position B. */
		case 0x4C:				/* Commanded throttle. */
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			break;
		case 0x51:
			can_cmd.dataA = 8;	/* Running on 'lectricity! */
			count = 3;
			break;
		case 0x5C:	/* Engine oil temp A-40, -40C..+215C, report motor temp. */
			/* The calculation is for a MCP9701 sensor.
			 * It outputs 400mV at 0C, and increases by 19.5mV/C. */
			if (raw_adc[6] < 1000)
				can_cmd.dataA = (raw_adc[6]+78)>>2;
			else		/* Report 0C if sensor missing. */
				can_cmd.dataA = 40;
			count = 3;
			break;
			/* A few more are easily added once we have these tested. */
			/* 0x4D Run time with MIL on. */
			/* 0x4E Time since trouble codes cleared. */
			/* 0x5A relative accelerator pedal position. */
			/* 0x5B hybrid pack remaining life. */
			/* 0x5E fuel rate. */
			/* 0x5F emissions requirements for this vehicle. */
		}
		break;
	case 2: switch(can_cmd.pid) {
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0x40;	/* Hand calculated. */
			can_cmd.dataB = 0x00;
			can_cmd.dataC = 0x00;
			can_cmd.dataD = 0x00;
			count = 6;
			break;
		case 0x02:				 /* Freeze data at last DTC  */
			can_cmd.dataA = 0x00;		/* No DTC reported. */
			can_cmd.dataB = 0x00;
			count = 4;
			break;
		}
		break;
	case 3: {
		/* Return a list Diagnostic Trouble Codes (DTCs)
		 * P0110-4 Intake air circuit malfunctionn
		 * P0115-9 Engine coolant temperature
		 * P0120-4 Thottle position sensor/switch (TPS)
		 * P0195-9 Engine oil temperature sensor
		 * +0  Circuit malfunction
		 * +1  Circuit range (e.g. gaps in response) problem
		 * +2/+3/+4  high/low/intermittent input
		 * With more than 3 DTCs, we must contruct an ISO-TP sequence.
		 * See the VIN response for an example.  For now we keep it simple
		 * and never have a multiple frame response.
		 */
		can_cmd.mode = 0x01;	  	/* First DTC byte 0 and 1 */
		can_cmd.pid = 0x20;
		can_cmd.dataA = 0x01;		/* Start of second DTC */
		count = 2;
		return count;				/* Avoid munging the PID field. */
	}
	case 9: switch(can_cmd.pid) {
			/* The VIN and calibration ID are the only times we need
			 * multi-frame messages, thus we shift the header rather
			 * than have general-purpose code.  */
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0x40;	  	/* 01: -- 02 -- -- -- -- -- -- */
			can_cmd.dataB = 0x00;		/* 09: -- -- -- -- -- -- -- -- */
			can_cmd.dataC = 0x00; 		/* 11: -- -- -- -- -- -- -- -- */
			can_cmd.dataD = 0x00;		/* 19: -- -- -- -- -- -- -- -- */
			count = 6;
			break;
		case 0x02:
			can_cmd.mode = 20;		  	/* Bytes in response. */
			can_cmd.pid = 0x49;			/* Shift the mode and PID over. */
			can_cmd.dataA = 0x02;
			VIN.delay = 255;			/* Mark as wait for flow control. */
			VIN.frame_idx = 1;			/* First of 5 lines */
			can_cmd.dataB = 1;			/* Always 1: Only a single VIN */
			VIN.msg_cnt = 17;			/* Strlen(VIN_str). */
			VIN.message = VIN_str;
			/* Only 3 bytes in the first response */
			for (count = 0; count < 3; count++)
				((char *)&(can_cmd.dataC))[count] = *VIN.message++;
			VIN.msg_cnt -= count;
			count = 20;
			break;
		}
		break;
#if ! defined(COUGAR)
		/* Mode A0 -- our vendor-specific/proprietary parameter set.
		 * We generate a response message that includes the original command.
		 */
	case 0xA0: switch(can_cmd.pid) {
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0x3C;	  	/* 01: -- -- 03 04 05 06 -- -- */
			can_cmd.dataB = 0x1F;		/* 09: -- -- -- 0C 0D 0E 0F 10 */
			can_cmd.dataC = 0xFF; 		/* 11: 11 12 13 14 15 16 17 18 */
			can_cmd.dataD = 0xFE;		/* 19: 19 1A 1B 1C 1D 1E 1F -- */
			count = 6;
			break;
		case 0x03:		/* Remote throttle position.  */
			if (can_cmd.cnt == 4) {
				throttle.remote = can_cmd.dataA;
				throttle.remote_valid = can_cmd.dataB;
				throttle.remote_time = clock_1msec;
				count = 4;
			}
			break;
		case 0x04:		/* Fault bits.  */
			can_cmd.dataA = fault_bits;
			can_cmd.dataB = 0;			/* All upper fault bits are zero. */
			count = 4;
			break;
		case 0x05:		/* Max motor current.  */
			can_cmd.dataA = m_current_peak >> 8;
			can_cmd.dataB = m_current_peak;
			count = 4;
			break;
		case 0x06:
			can_cmd.dataA = fault.cycles_off >> 8;
			can_cmd.dataB = fault.cycles_off;
			count = 4;
			break;
		case 0x0C:		/* Set target engine RPM, in quarter-RPMs.  */
			if (can_cmd.cnt == 4 && can_cmd.dataA < 0x60) {
				PTO_fast_idle = (can_cmd.dataA << 8) + can_cmd.dataB;
				tach_target_qrpm = PTO_fast_idle;
				count = 4;
			}
			break;
		case 0x0D:		/* Set maximum RPM, in quarter-RPMs.  */
			if (can_cmd.cnt == 4 && can_cmd.dataA < 0x60) {
				tach_redline_qrpm = (can_cmd.dataA << 8) + can_cmd.dataB;
				count = 4;
			}
			break;
		case 0x0E:		/* Get target RPM, quarter-RPMs.  */
			can_cmd.dataA = tach_target_qrpm >> 8;
			can_cmd.dataB = tach_target_qrpm & 0xFF;
			count = 4;
			break;
		case 0x0F:		/* Get maximum RPM, quarter-RPMs.  */
			can_cmd.dataA = tach_redline_qrpm >> 8;
			can_cmd.dataB = tach_redline_qrpm & 0xFF;
			count = 4;
			break;
		default:
			if (can_cmd.pid >= 0x10 && can_cmd.pid < 0x20) {
				/* Read raw sensor value.  */
				uint8_t adc_channel = can_cmd.pid - 0x10;
				can_cmd.dataA = raw_adc[adc_channel] >> 8;
				can_cmd.dataB = raw_adc[adc_channel] & 0xFF;
				count = 4;
			}
			break;
		}
		break;
#endif
	}
	if (count) {
		can_cmd.cnt = count;
		can_cmd.mode |= 0x40;
	} else {					/* mode $7F: reject message */
		can_cmd.dataB = 0x11;	/* Unsupported mode */
		can_cmd.dataA = can_cmd.pid;
		can_cmd.pid = can_cmd.mode;
		can_cmd.mode = 0x7F;
		can_cmd.cnt = 4;
	}
	return count;
}

void CAN_Rx_frame(unsigned char buffer)
{
	uint8_t count, i;

#if 1									/* Debugging version. */
	if (can_verbose > 2) {
		uint8_t addr[5];
		CAN_CS_ENABLE;
		SPI_Transmit(buffer ? MCP_ReadRx1 : MCP_ReadRx0);
		for (i = 0; i < 5; i++)			/* Read the header */
			addr[i] = SPI_Transmit(0x00);
		for (i = 0; i < 8; i++) {			/* Read the frame data. */
			((unsigned char *)&can_cmd)[i] = SPI_Transmit(0x00);
		}
		CAN_CS_DISABLE;
		strcpy_P(uart_str, PSTR(" CAN Rx 0xHHH HH HH HH.\n\r"));
		u16x_to_str(&uart_str[10], (addr[0]<<3) | (addr[1]>>5), 3);
		u16x_to_str(&uart_str[14], can_cmd.cnt, 2);
		u16x_to_str(&uart_str[17], can_cmd.mode, 2);
		u16x_to_str(&uart_str[20], can_cmd.pid, 2);
		uart_putstr();
	} else {					/* Only read the message bytes. */
		CAN_CS_ENABLE;
		SPI_Transmit(buffer ? MCP_ReadRx1Payload : MCP_ReadRx0Payload);
		for (i = 0; i < 8; i++) {			/* Read the frame data. */
			((unsigned char *)&can_cmd)[i] = SPI_Transmit(0x00);
		}
		CAN_CS_DISABLE;
	}	
#else
	CAN_CS_ENABLE;
	SPI_Transmit(buffer ? MCP_ReadRx1Payload : MCP_ReadRx0Payload);
	for (i = 0; i < 8; i++) {			/* Read the frame data. */
		((unsigned char *)&can_cmd)[i] = SPI_Transmit(0x00);
	}
	CAN_CS_DISABLE;
#endif
	if (can_cmd.cnt < 7) {		/* The normal case, a Single Frame */
		count = OBD2_respond();
		if (count) {
			/* Most responses are simple Single Frames.  Long responses
			 * have filled in the flow control structure. */
			if (count >= 8) {
				can_cmd.cnt = 0x10;		/* 0x10 | (count>>8) */
				can_cmd.mode = count;
				VIN.id = CAN_ECU_ID;
			}
			can_cmd.id = CAN_ECU_ID;
			can_transmit();
		}
	} else if ((can_cmd.cnt & 0xF0) == 0x30) { /* A flow control frame. */
		switch (can_cmd.cnt & 0x0F) {
		case 0:
			VIN.window = can_cmd.mode;
			VIN.delay = can_cmd.pid;
			/* VIN.last_send = counter_10k; */
			break;
		case 1:							/* Wait */
			VIN.delay = 255;
			break;
		case 2:	default:				/* Overflow/invalid: abort */
			VIN.msg_cnt = 0;
			break;
		}
		if (can_verbose > 3) {
			serprintf(PSTR("  CAN flow control %2x %d %d %d.\n\r"),
					  can_cmd.cnt, can_cmd.mode, can_cmd.pid, can_cmd.dataA);
		}
	}
	return;
}

/* Poll the CAN bus.
 * This is called as frequently as possible from the main loop.
 * It may be used as the main interactive loop or for deferred interrupt
 * processing.
 */
void CAN_poll(void)
{
	static int8_t showed_message = 0;
	uint8_t status, iflags = 0;

	if (CAN_enabled == 0) {
		if ( ! CAN_reset_done)
			can_setup();
		return;
	}

#if defined(COUGAR)
	/* Keep our local OBD time in seconds.
	 * Robust and non-critical, so do not bother locking.
	 */
	if (counter_1k - local_clock > 976) {
		tm_seconds++;
		local_clock += 976;
	}
#endif

	status = can_get_status();

	/* Borked or boring. */
	if (status == 0x00)
		return;
	if (status == 0xFF) {
		CAN_enabled = 0;
		return;
	}

	/* This will be cleaned up after we verify that Cougar works. */
	if (can_verbose > 1  && showed_message < 20) {
		showed_message++;
		strcpy_P(uart_str, PSTR("CAN_poll() Status is 0x--\r\n"));
		u16x_to_str(&uart_str[23], status, 2);
		uart_putstr();
	}

	/* Check if any interrupt bit is set.
	 * Rx interrupts are acknowledged by ReadRxBuf command
	 */
	if (status) {
		if (status & 0x01) {			/* Packet in RxBuf0 */
			CAN_Rx_frame(0);
		}
		if (status & 0x02) {			/* Packet in RxBuf1 */
			CAN_Rx_frame(1);
		}
		if (status & 0x04) {			/* Packet remains in TxBuf0 */
			/* Debugging code -- remove later. */
			CAN_CS_ENABLE;
			SPI_Transmit(MCP_Read);
			SPI_Transmit(0x30);			/* Get TxBuf0Ctrl status pg 18. */
			status = SPI_Transmit(0x00);
			CAN_CS_DISABLE;
			if (can_verbose) {
				strcpy_P(uart_str, PSTR("  Tx0 Status is 0x--\r\n"));
				u16x_to_str(&uart_str[18], status, 2);
				uart_putstr();
			}
#if 0
			if (status != 0)
				CAN_enabled = CAN_reset_done = 0;
#endif
		}
		if (status & 0x08) {			/* Packet sent from TxBuf0 */
			txbuf[0].TxCtrl &= ~0x08;
			iflags |= 0x04;
		}
		if (status & 0x20) {			/* Packet sent from TxBuf1 */
			txbuf[1].TxCtrl &= ~0x08;
			iflags |= 0x08;
		}
		if (status & 0x80) {			/* Packet sent from TxBuf2 */
			txbuf[2].TxCtrl &= ~0x08;
			iflags |= 0x10;
		}
	}
	if (iflags) {
		CAN_CS_ENABLE;
		SPI_Transmit(MCP_BitMod);
		SPI_Transmit(0x2C);			/* Interrupt flags */
		SPI_Transmit(iflags);		/* Select the bits to modify. */
		SPI_Transmit(0x00);			/* Clear them as interrupt sources. */
		CAN_CS_DISABLE;
	}

	if (VIN.msg_cnt != 0 && VIN.delay != 255) {
		/* More multi-frame (only VIN!) lines remain to be sent. */
		uint8_t i;
#if 0
		/* Check delay, if the timer is running. */
		if (timer_10KHz &&			/* Timer must be running */
			(VIN.delay <= 127 &&
			 timer_10KHz - VIN.last_send > VIN.delay*10) ||
			(VIN.delay > 0xF0 && VIN.delay <= 0xF9 &&
			 timer_10KHz - VIN.last_send > VIN.delay - 0xF0 + 1))
#endif
			can_cmd.cnt = 0x20 | (VIN.frame_idx & 0x0F);
		for (i = 0; i < 7  && --VIN.msg_cnt > 0; i++)
			((char *)&(can_cmd.mode))[i] = *VIN.message++;
		if (can_transmit() != 0) {
			VIN.message -= i;
			VIN.msg_cnt += i;
		} else {
			VIN.frame_idx++;
		}
	}

	return;
}

/* This is periodically called from the main loop to report operating
 * conditions, as well as to generate traffic for monitors that expect
 * to autobaud.
 */
static uint8_t report_counter = 0;
	
void CAN_heartbeat(void)
{
	if ( ! CAN_enabled)
		return;
#if 1
	if (can_verbose && (report_counter++ & 7) == 0) {
		uint8_t regs[4], i;
		CAN_CS_ENABLE;
		SPI_Transmit(MCP_Read);
		SPI_Transmit(0x1C);				/* TxErrCnt/RxErrCnt/CANStat/CANCtrl */
		for (i = 0; i < 4; i++)
			regs[i] = SPI_Transmit(0x00);
		CAN_CS_DISABLE;
		strcpy_P(uart_str, PSTR("CAN status is TxErr:DDD RxErr:DDD "
								"0xXX 0xXX.\r\n"));
		u16_to_str(&uart_str[20], regs[0], 3);
		u16_to_str(&uart_str[30], regs[1], 3);
		u16x_to_str(&uart_str[36], regs[2], 2);
		u16x_to_str(&uart_str[41], regs[3], 2);
		uart_putstr();
		CAN_show_registers(0x20);
	}
#endif
	can_cmd.cnt = 0x07;
	can_cmd.mode = 0x10;		/* Our own packet type. */
	can_cmd.pid = 0x23;
	can_cmd.dataA = rt_data.throttle_ref >> 1; 	/* Throttle position */
	can_cmd.dataB = config.motor_sc_amps;
	can_cmd.dataC = tach_get_QRPM() >> 7;		/* Scaled RPM, 0-8K. */
	can_cmd.dataD = (125 * (rt_data.raw_hs_temp - 477)) >> 8;
	can_cmd.unnamed = fault_bits; 			/* Longer than OBD2 responses. */
	can_cmd.id = CAN_PHY_ADDR;
	can_transmit();
	return;
}


/*
 * Local variables:
 *  compile-command: "make"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
