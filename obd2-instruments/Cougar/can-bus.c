/* can-bus.c: CAN interface for the VVVVroom motor controller. */
/*
 * CAN bus interface and reporting.
 * This file has the code for responding to CAN bus messages, reporting
 * state, status, statistics and setting the configuration.
 *
 * We respond to SAE J1979 PIDs as well as our own "manufacture specific"
 * PIDs.
 *
 * This code currently implements most relevant OBD2 PIDs, with opportunity
 * remaining to report additional values.
 *
 * We use an 8MHz clock from an AVR timer at max rate.  This is high enough
 * to support the common 250K and 500K NBR (Nominal Bit Rate) CAN bus speeds.
 * The controller needs at least 10 cycles per bit to sample and
 * resynchronize, so supporting a future 1M NBR requires a local crystal.
 *
 * We always configure to 500Kbps and start broadcasting, expecting that
 * other devices will be silently waiting to autobaud against our traffic.
 *
 * References for understanding this code
 *  MCP2515 datasheet rev D, DS21801D (default for page number references)
 *  http://en.wikipedia.org/wiki/On-board_diagnostics
 *  http://en.wikipedia.org/wiki/OBD-II_PIDs
 *  http://www.semiconductors.bosch.de/pdf/can2spec.pdf
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
 * There must always be another active CAN device to successfully
 * transmit.  If using an ELM327 v1.42b and later as the only other bus
 * device, turn silent monitoring off with "AT CSM1".  Otherwise it will
 * only provide the ACK to messages it is listening to.
 */
static const char versionA[] =
"can-bus.c: 1/3/2011 Copyright Donald Becker\n";

#if defined(__AVR_ATmega168__)
#define MEGA168
#endif
#if defined(__AVR_ATmega1280__)
#define MEGA1280
#endif

/* We need very few library routines, mostly AVR-specific hardware
 * macros. */
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(IOM8)
#include <avr/iom8.h>
#elif defined(MEGA168)
#include <avr/iom168.h>
#else
#include <avr/iom1280.h>
#endif

#if defined(COUGAR)
#include "cougar-export.h"
#include "cougar.h"
#ifndef tach_get_QRPM
#define tach_get_QRPM() 1234*4
#endif
/* We need to deduce our own clock from counter_1k. */
uint16_t local_clock, tm_seconds;
#else
#include "vvvvroom.h"
#endif
#include "can.h"

/* The default VIN, used if not set from the EEPROM config.
 * A VIN is typically only 17 ASCII characters. */
char VIN_str[20] = "WBAAA1305H2320124";

#define PB_CAN_CS DDB0		/* Abitrary digital pin used for active-low /CS */
/* GCC-avr compiles these macros to single instructions. */
#define CAN_CS_ENABLE	PORTB &= ~(1 << PB_CAN_CS);		/* Chip select low */
#define CAN_CS_DISABLE	PORTB |= (1 << PB_CAN_CS);		/* Chip select high */
#define SPI_CLK_DIV	(0 << SPR0) /* 0,1,2,3 is divisor by 4,16,64,128  */

/* Our CAN ID for operational (priority, non-diagnostic) frames. */
#define CAN_PHY_ADDR 0x420

/* Symbolic constants for the MCP2515 CAN controller.
 * I omit most single-use symbolic names: they don't add clarity, and
 * introduce an opportunity for errors when writing and verifying the code.*/

/* Named constants for the configuration registers.
 * These are only used once, but are a bit involved.
 * Most fields describe how the clock is used by the receiver to detect
 * edges and bit values.
 * We set the prescaler to '0', which means divides by 2.
 * The sync segment is fixed at 1
 * The propagation segment is 0, which means 1 TQ
 * The phase 1 segment is 2, which means 3TQ (minimum 1TQ)
 * The phase 2 segment is 2, which means 3TQ (minimum 2TQ)
 */
/* Settings for an 8MHz clock, 500Kbps NBR, or 4MHz/250Kbps. */
#define MCP_CNF1  0x00 		/* Prescaler at minimum, 4MHz TQ. */
#define MCP_CNF2  0x90		/* BTLMODE, PHSEG=2, PRSEG=0 */
#define MCP_CNF3  0x42		/* PHSEG2=2, wake-up filter, CLKOUT pin is clk */

#if defined(MCP_CLK_10MHz)
/* Settings for an 10MHz clock, 500KHz NBR. */
#define MCP_CNF1  0x00 		/* Prescaler at minimum, 5MHz TQ */
#define MCP_CNF2  0x98		/* BTLMODE, PHSEG=3, PRSEG=0 */
#define MCP_CNF3  0x03		/* PHSEG2=3, no wake-up, CLKOUT pin is clk */
#endif

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

#define COUGAR_SETVAR
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
	while( ! (SPSR & (1<<SPIF)))
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
 * This might be Mega 1280 specific -- I will generalize when it has been
 * tested.
 *  					SS SCK MOSI MISO
 *  AT90USB82/162 		PB0	PB1 PB2 PB3
 *  ATmega48/88/168/328 PB2	PB5 PB3 PB4
 *  ATtiny8			 	-	- 	PA6	PA5
 * We set these SPI pins on port B as outputs:  MOSI SCK SS
 * And, grrrr, a zillion pointless defines e.g. 'DDB2' and not a single one
 * to make chip-independent SPI code possible e.g. for MOSI/MISO/SCK/SS.
 */

uint8_t can_reset(void)
{
	uint8_t i;

	DDRB |= ((1<<DDB2)|(1<<DDB1)|(1<<PB_CAN_CS));
	CAN_CS_DISABLE;

	/* Turn on: SPI enable, MSB first, 0,0, Master, F_OSC/4. */
	SPCR = (1 << SPE) | (1 << MSTR) | SPI_CLK_DIV;
	SPSR = 1 << SPI2X;				/* Double clock to 8MHz */

	/* Send a reset as a stand-alone command and monitor completion. */
	/* This is SPI_Transmit(MCP_Reset) with a fail check. It typically
	 * takes 3-4 loops to complete. */
	CAN_CS_ENABLE;
	SPDR = MCP_Reset;
	for (i = 255; i > 0; i--) {
		if (SPSR & (1<<SPIF))
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
void can_show_registers(int8_t base)
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
	if (CAN_reset_done == 0 && can_reset() == 0) {
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
int OBD2_respond(void)
{
	uint8_t count = 0;

	switch(can_cmd.mode) {
	case 1: switch(can_cmd.pid) {		/* ShowCurrentData */
		case 0x00:				/* Respond with bitmap of PIDs supported  */
			can_cmd.dataA = 0xB8;	  	/* 01 -- 03 04 05 -- -- -- */
			can_cmd.dataB = 0x12;		/* -- -- -- 0c -- -- 0f -- */
			can_cmd.dataC = 0x80; 		/* 11 -- -- -- -- -- -- -- */
			can_cmd.dataD = 0x13;		/* -- -- -- 1c -- -- 1f 20 */
			count = 6;
			break;
		case 0x01:
			can_cmd.dataA = 0;	/* No trouble light, no error codes. */
			can_cmd.dataB = 0x0F;	/* Nothing to see here.  Move along now. */
			can_cmd.dataC = 0xFF;
			can_cmd.dataD = 0x00;
			count = 6;
			break;
		case 0x03:
			can_cmd.dataA = 2;	/* We pretend to be closed loop. */
			can_cmd.dataA = 0;	/* with no second fuel system */
			count = 4;
			break;
		case 0x04:	/* Calculated Engine load 0..255, we use amps. */
			/* Percentage of available torque, not rated torque. */
			can_cmd.dataA = config.motor_sc_amps;
			count = 3;
			break;
		case 0x05:	/* Coolant temp A-40 -40C..+215C, we use heatsink temp. */
			/* Assume a LM335 where  Kelvin = 100 * 5V * (count / 1024)
			 * Our baseline temp is -40C or 233.15 Kelvin, or 2.33V
			 * which is an A/D reading of about 477
			 * DataA = ((500 * rt_data.raw_hs_temp)/ 1024) - 233  */
			can_cmd.dataA = (125 * (rt_data.raw_hs_temp - 477)) >> 8;
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
		case 0x0F:	/* Intake temp A-40 -40C..+215C, use fan intake. */
			/* See coolant temp calculations above. */
			can_cmd.dataA = (125 * (rt_data.raw_hs_temp - 477)) >> 8;
			count = 3;
			break;
		case 0x11:	/* Absolute throttle pos. 0..255, we use pedal position. */
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			count = 3;
			break;
		case 0x1c:
			can_cmd.dataA = 9;	/* We respond to EOBD, OBD and OBD II */
			count = 3;
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
			can_cmd.dataD = 0x00; 		/* None, not even 0x60 */
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
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			break;
		case 0x47:				/* Absolute throttle position B. */
			can_cmd.dataA = rt_data.throttle_ref >> 1;
			break;
		case 0x51:
			can_cmd.dataA = 8;	/* Running on 'lectricity! */
			count = 3;
			break;
			/* A few more are easily added once we have these tested. */
			/* 0x4C Commanded throttle */
			/* 0x4D Run time with MIL on. */
			/* 0x4E Time since trouble codes cleared. */
			/* 0x5A relatie accelerator pedal position. */
			/* 0x5B hybrid pack remaining life. */
			/* 0x5C engine oil temperature. */
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
	case 9: switch(can_cmd.pid) {
		case 0x02:
			/* The VIN is the only time we need multi-frame messages, thus
			 * we shift the header rather than have general-purpose code.  */
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
	} else {
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
			if (count < 8) {
				can_cmd.cnt = count;
				can_cmd.mode |= 0x40;
			} else {
				can_cmd.cnt = 0x10;		/* 0x10 | (count>>8) */
				can_cmd.mode = count;
			}
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
			strcpy_P(uart_str, PSTR("  CAN flow control xHH DDD DDD DDD.\n\r"));
			u16x_to_str(&uart_str[20], can_cmd.cnt, 2);
			u16_to_str(&uart_str[23], can_cmd.mode, 3);
			u16_to_str(&uart_str[27], can_cmd.pid, 3);
			u16_to_str(&uart_str[31], can_cmd.dataA, 3);
			uart_putstr();
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
		u16x_to_str(&uart_str[26], status, 2);
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

/* This may be periodically called from the main loop to report operating
 * conditions, as well as to generate traffic for monitors that expect to
 * autobaud.  Do not enable this unless there is at least one other active CAN
 * bus node, as the lack of a bus ACK will shut down the transmitter.
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
		can_show_registers(0x20);
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
