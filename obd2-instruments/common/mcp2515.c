/* CAN-mcp2515.c: CAN interface for the VVVVroom motor controller. */
/*
 * CAN bus driver for the MCP2515 CAN controller.
 * This file has the code for responding to CAN bus messages, reporting
 * state, status, statistics and setting the configuration.
 *
 * We use an 8MHz clock from either an AVR timer output or a local crystal.
 * This is high enough to support the common 250K and 500K NBR
 * (Nominal Bit Rate) CAN bus speeds, but not the maximum 1M.
 * The controller needs at least 10 cycles per bit, preferably 16, to sample
 * and resynchronize, so supporting 1M NBR requires a 16MHz crystal.
 *
 * We always configure to 500Kbps and start broadcasting, expecting that
 * other devices will be silently waiting to autobaud against our traffic.
 *
 * References for understanding this code
 *  MCP2515 datasheet rev D, DS21801D (default for page number references)
 *  http://en.wikipedia.org/wiki/OBD-II_PIDs
 *  http://en.wikipedia.org/wiki/ISO_15765-2
 *  http://www.semiconductors.bosch.de/pdf/can2spec.pdf
 *  SAE-J2190
 */

static const char versionA[] =
"mcp2515.c: $Id: mcp2515.c becker $ Copyright Donald Becker\n";

#if defined(STM32)
/* The STM32 doesn't need the awkward Harvard architecture hacks of the AVR. */
#include <armduino.h>
extern void *memset(void *s, int c, long n);
#else
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#if defined(COUGAR)
/* Tweaks to work with the Cougar firmware. */
#include "cougar-export.h"
#include "cougar.h"
#endif
#endif

#include "can.h"

extern int serprintf(const char *format, ...)
	__attribute__ ((format(printf, 1, 2)));;

/* Our CAN ID for operational (non-diagnostic, higher priority) frames. */
#define CAN_PHY_ADDR 0x420

/* State of the Tx and Rx buffers on the MCP2515.
 * It takes five extra bytes, 80 clock cycles, to re-write the Tx header.
 * We record the previous value to check if its unchanged.
 */
struct tx_buf_state {
	uint8_t  TxCtrl;				/* TxBnCTRL contents. */
	uint16_t TxID;					/* TxBnSIDL address. */
} txbuf[3];

/* Try to put a little abstraction on the bit banging and very
 * different hardare.
 */
#if defined(STM32)
/* Any GPIO output may be used for the active-low /CS.  Set to PA15 */
#define SPI_CAN_CS 15
#define GPIO_SPI_BRR GPIOA_BRR
#define GPIO_SPI_BSRR GPIOA_BSRR
#define CAN_CS_ENABLE \
	GPIO_SPI_BRR = (1 << SPI_CAN_CS);		/* Chip select low */
#define CAN_CS_DISABLE \
	GPIO_SPI_BSRR = (1 << SPI_CAN_CS);		/* Chip select high */
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

/* Controller register initialization.
 * Uglier than the earlier version, but doing it this way shrinks the
 * size from 0x242 to 0x1xx bytes.
 */

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
#if defined(STM32)
	int result;
	CAN_CS_ENABLE;
#if 0
	/* A better sequential transmit routine for the STM32, which has
	 * both Tx-empty and Rx-filled status bits.
	 * It has a termination bug.  Better, it should be rewritten for DMA.
	 */
	SPDR = *data++;
	while (--size > 0) {
		int i = 0;
		while ( ! (SPSR & SPI_TXE) && ++i > 5)
			;
		SPDR = *data++;
		while ( ! (SPSR & SPI_RXNE) && ++i > 5)
			;
		result = SPDR;
		if (i > 2)
		serprintf("Exceeded iteration check %d with %d bytes left.\n",
				  i, size);
	}
	while ( ! (SPSR & SPI_RXNE))
		;
	result = SPDR;
#else
	do {
		SPDR = data;
		while ( ! (SPSR & TXE))
			;
		SPI_Transmit(pgm_read_byte(data++));
	} while (--size != 0);
#endif
	CAN_CS_DISABLE;
#else  /* AVR */
	CAN_CS_ENABLE;
	do {
		SPI_Transmit(pgm_read_byte(data++));
	} while (--size != 0);
	CAN_CS_DISABLE;
#endif
}

/* Initialize the SPI interface.
 * For the STM32 processors
 *			SS   SCK  MOSI MISO
 *  SPI1	PA4  PA5  PA7  PA6
 *  SPI1-A  PA15 PB3  PB5  PB4
 *  SPI2	PB12 PB13 PB15 PB14
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
	/* Configure SPI: SPI enable, 8 bit, MSB first, 0,0, Master, F_OSC/4. */
	SPI_CR1 = 0x4300 | SPI_SPE | SPI_CLK_DIV | SPI_MSTR;
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
	CAN_enabled = 0;
	CAN_reset_done = 1;
	memset(txbuf, 0, sizeof txbuf);
	return i;
}

uint8_t CAN_dev_start(void)
{
	uint8_t i, status;

	/* Wait until a read status command works.
	 * This typically takes 4 loops after a reset, so allow 10x.*/
	for (i = 0; i < 40; i++) {
		status = mcp2515_get_status();
		if (status != 0xFF) {
			CAN_enabled = 1;
			break;
		}
	}

	if (status == 0xFF)
		serprintf(PSTR("MCP2515 reset failed.\n"));

	/* Continue setup anyway. */
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
	SPI_Transmit((0x7FF << 5)&0xFF);
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

/* Reset the device, leaving it offline and ready to be initialized. */
uint8_t CAN_dev_stop(void)
{
	CAN_reset_done = 0;

	CAN_CS_ENABLE;
	SPI_Transmit(MCP_Write);
	SPI_Transmit(0x0F);					/* CANCTRL */
	SPI_Transmit(0x20);					/* Set sleep mode. */
	CAN_CS_DISABLE;
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

/* Transmit the frame in can_cmd.
 * At minimum we must write TxBnSIDH TxBnSIDL TxBnDLC and the
 * message itself.
 *
 * We are extra clever and scan for an empty Tx buffer with a matching
 * previuosly-written header.  This is probably foolish, as it could
 * result in out-of-order multi-frame responses, and priority inversion.
 *
 * The original, trivial version with fixed ID assignment is probably better.
 */
int8_t CAN_dev_transmit(void)
{
	uint8_t i, skip_header, buf_num;
	uint16_t SID = can_cmd.id;

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
		SPI_Transmit(SID >> 8);			/* TXBnSIDH, SID is pre-shifted. */
		SPI_Transmit(SID);				/* TXBnSIDL: No extended ID */
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
	if (can_verbose > 3)
		serprintf(PSTR(" CAN Tx %3x %2x %2x %2x %2x %2x %2x %2x.\n"),
				  SID>>5, can_cmd.cnt, can_cmd.mode, can_cmd.pid,
				  can_cmd.dataA, can_cmd.dataB, can_cmd.dataC, can_cmd.dataD);

	return 0;
}


void mcp2515_Rx_frame(unsigned char buffer)
{
	uint8_t i;

#if 1									/* Debugging version. */
	if (can_verbose > 2) {
		uint8_t addr[5];
		CAN_CS_ENABLE;
		SPI_Transmit(buffer ? MCP_ReadRx1 : MCP_ReadRx0);
		for (i = 0; i < 5; i++)			/* Read the header */
			addr[i] = SPI_Transmit(0x00);
		can_cmd.id = *(uint16_t *)addr;
		for (i = 0; i < 8; i++) {			/* Read the frame data. */
			((unsigned char *)&can_cmd)[i] = SPI_Transmit(0x00);
		}
		CAN_CS_DISABLE;
		serprintf(PSTR(" CAN Rx %3x %2x %2x %2x.\n"),
				  (addr[0]<<3) | (addr[1]>>5), can_cmd.cnt,
				  can_cmd.mode, can_cmd.pid);
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
	CAN_process_rx_frame();
}

uint8_t mcp2515_get_status(void)
{
	uint8_t status;
	CAN_CS_ENABLE;
	SPI_Transmit(MCP_ReadStatus);
	status = SPI_Transmit(0xFF);		/* 0xFF: detect non-response. */
	CAN_CS_DISABLE;
	return status;
}

/* Poll the CAN bus.
 * This is called as frequently as possible from the main loop.
 * It may be used as the main interactive loop or for deferred interrupt
 * processing.
 */
void CAN_dev_poll(void)
{
	static int8_t showed_message = 0;
	uint8_t status, iflags = 0;

	status = mcp2515_get_status();

	/* Borked or boring. */
	if (status == 0x00)
		return;
	if (status == 0xFF) {
		CAN_enabled = 0;
		return;
	}

	/* This will be cleaned up after development is complete. */
	if (can_verbose > 1  && showed_message < 20) {
		showed_message++;
		serprintf(PSTR("mcp2515_poll() Status is %2x\n"), status);
	}

	/* Check if any interrupt bit is set.
	 * Rx interrupts are acknowledged by ReadRxBuf command
	 */
	if (status) {
		if (status & 0x01) {			/* Packet in RxBuf0 */
			mcp2515_Rx_frame(0);
		}
		if (status & 0x02) {			/* Packet in RxBuf1 */
			mcp2515_Rx_frame(1);
		}
		if (status & 0x04) {			/* Packet remains in TxBuf0 */
			/* Debugging code -- remove later. */
			CAN_CS_ENABLE;
			SPI_Transmit(MCP_Read);
			SPI_Transmit(0x30);			/* Get TxBuf0Ctrl status pg 18. */
			status = SPI_Transmit(0x00);
			CAN_CS_DISABLE;
			if (can_verbose) {
				serprintf(PSTR("  Tx0 Status is %2x\n"), status);
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

	return;
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

	serprintf(PSTR(" CAN registers %2x:"), base);
	for (i = 0; i < 16; i++)
		serprintf(PSTR(" %2x"), result[i]);
	serprintf(PSTR("\n"));
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
