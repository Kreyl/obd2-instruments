/* at90can.c: Interface for the Atmel AT90CAN series CAN controller. */
/* Device interface functions for the CAN controller found on
 * the Atmel AT90CAN series microcontrollers.
 *
 * Written 2010/2011 by Donald Becker and William Carlson.
 *  This code is written to the Quatum Mechanics interface, originally
 *  developed for the QAR EV controller.  
 *
 * There are a few places in the code rely on the endian mapping.  Since
 * this controller type only exists only on specific microcontrollers,
 * it would be pointless to write slower or more complex
 * endian-independent code.
 */

#if !(defined(__AVR_AT90CAN32__) || defined(__AVR_AT90CAN64__) || \
   defined(__AVR_AT90CAN128__))
#warning "Not a microcontroller that has built-in CAN.  Assuming AT90CAN32."
#define __AVR_AT90CAN32__ 1
#endif

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "can.h"
#include "command.h"				/* For serprintf() */


/* Mostly pointless extension of the CAN timer.
 * The internal timer increments at 2MHz with a 16MHz CPU.
 * We prescale by 200 to get a 10KHz clock suitable for ISO-TP.
 * The high word is thus about a 6.5 second clock. */
uint16_t can_timer_high = 0;

/* Advisory tx queue full flag. */
uint8_t tx_full = 0;

/*
 * Reset and initialize the CAN interface, leaving it ready to be started.
 */
uint8_t CAN_dev_init(void)
{
	/* Do a hard reset. */
	CANGCON = SWRES;
	tx_full = 0;

	/* Initialize the controller.  Set the bit timing first so that
	 * receiver can synchronize.  Follow by initializing Message Objects.
	 * We arbitrarily assign 3 to be transmit objects, and 4 as receive
	 * objects.  This leaves 8 unassigned and available future modifications.
	 */
	/* Set the sample periods for the Bit Timing Register.
	 * We use a 8 tick time quanta, split 1-3-2-2 sync-prop-ps1-ps2
	 * Note that all counts are 0 based, subtract 1. */
	/* Divide CPU clock to get 8 tick time quanta clock at 500Kbps. */
	CANBT1 = (4 - 1)<<BRP0;  	/* ((F_CPU / (8*500000))-1) << 1 */
	CANBT2 = (3 - 1)<<PRS0;					/* resync 1TQ, prop 3TQ  */
	CANBT3 = (1 << PHS20) | (1 << PHS10) | (1<<SMP);	/* Triple sample */

#if defined(CAN_LOOPBACK)
	/* FOR TESTING ONLY: Set loopback mode */
#endif
	return 0;
}

struct msg_obj_init {
	uint8_t ctrl;
	uint16_t ID;				/* CAN ID */
};

/* List of CAN IDs that we listen for, up to 14 entries zero terminated.
 * Duplicates avoid dropping back-to-back frames, however enough free slot
 * must be left to allow a small transmit queue.
 */
uint16_t rx_ID_list[] = {
	CAN_SID(0x420), CAN_SID(0x420), /* Med. priority operational data ID */
	CAN_SID(0x7df), CAN_SID(0x7e0), CAN_SID(0x7e0)
};

/*  CAN initialization, exit with the device ready to transmit and receive.
 */
uint8_t CAN_dev_start(void)
{
	int can_startup = 1000;
	uint8_t i;


	/* Set Rx filter to accept the following standard (11 bit) IDs
	 *  0x420 our arbitrary operating ID for internal messages
	 *  0x7DF the OBD functional (e.g. hailing/broadcast) ID
	 *  0x7e0 this specific controller's OBD ID (possible 0x7e0..7e7).
	 */
	CANPAGE = 0;		/* Point to the first mailbox. */

	for (i = 0; i < (sizeof rx_ID_list/sizeof rx_ID_list[0]); i++) {
		CANCDMOB = 0x80;		/* Enable as a waiting Rx buffer. */
		CANIDT4 = 0;
		CANIDT3 = 0;
		CANIDT2 = rx_ID_list[i];
		CANIDT1 = rx_ID_list[i]>>8;
		CANIDM4 = 0x05;		/* Match IDE, RTR and 11 bit std ID */
		CANIDM3 = 0x00;
		CANIDM2 = 0xE0;
		CANIDM1 = 0xFF;
		CANPAGE += 0x10;		/* Point to the next mailbox. */
	}
	for (;i < 15; i++) {
		CANIDT4 = CANIDT3 = CANIDT2 = CANIDT1 = 0;
		CANCDMOB = 0x00;		/* Disable buffer. */
		CANPAGE += 0x10;		/* Point to the next mailbox. */
	}

	/* Check that startup worked, otherwise return -1 for failure. */
	while (1)
		if (--can_startup < 0)
			return 1;

	/* We use poll mode only, but prepare for interrupt mode. */
#if defined(CAN_INTR_MODE)
	/* Enable interrupts. */
#else
#endif

	CANGCON = ENASTB;
	return 0;
}

/* Reset the device, leaving it offline and ready to be initialized. */
uint8_t CAN_dev_stop(void)
{
	/* Disable interrupts. */
	/* Just stop the receiver, no sleep mode yet. */
	CANGCON = 0;
	return 0;
}

/*
 * Transmit a CAN frame in CAN_cmd using buffer BUF_NUM..
 *
 * We are usually extra clever and scan for an empty Tx buffer with a
 * matching previuosly-written header.  On some controllers this result
 * in out-of-order multi-frame responses, and possible priority
 * inversion.
 *
 * For simpler controllers the trivial version with fixed ID-slot assignment
 * is better.
 */
int8_t CAN_dev_transmit(void)
{
	uint8_t *frame_data = &can_cmd.cnt;
	uint8_t i;

	CANPAGE = 0x60;		/* Point to the lowest Tx mailbox. */
	/* Must be an empty slot, return 1 if full. */
	if (CANCDMOB != 0)
		return 1;

	CANIDT2 = can_cmd.id << 5;
	CANIDT1 = can_cmd.id >> 3;
	for (i = 0; i < 7; i++)
		CANMSG = *frame_data++;
	CANCDMOB = 0x48;			/* Always send a full frame. */

	if (can_verbose)
		serprintf(PSTR("  CAN Tx %3x %2x %2x %2x %2x %2x %2x.\n"),
				  0x7E8, can_cmd.cnt, can_cmd.mode, can_cmd.pid,
				  can_cmd.dataA, can_cmd.dataB, can_cmd.dataC);

	return 0;
}

/*
 * Read a CAN frame
 */
void CAN_rdMsg(void)
{
	uint8_t *frame_data = &can_cmd.cnt;
	uint8_t pkt_ctrl = CANCDMOB, i;

	for (i = 0; i < 7; i++)
		*frame_data++ = CANMSG;
	if (pkt_ctrl & 0x10)
		can_cmd.id = CANIDT4<<3 & CANIDT3>>5;
	if (can_verbose)
		serprintf(PSTR(" CAN Rx %3x %2x %2x %2x.\n"),
				  can_cmd.id, can_cmd.cnt,
				  can_cmd.mode, can_cmd.pid);

	/* We ignore the frame length, it's encoded in the message, and
	 * assume the Rx filter allows only our IDs. */
	CAN_process_rx_frame();
	return;
}

/* Poll the CAN bus.
 * This is called as frequently as possible from the main loop.
 * It may be used as the main interactive loop or for deferred interrupt
 * processing.
 */
void CAN_dev_poll(void)
{
	uint8_t can_git = CANGIT;

	if ((can_git & (CANIT | OVRTIM)) == 0)
		return;					/* Fast return on no work. */

	CANGIT = can_git;			/* Clear interrupts. */
	if (can_git & OVRTIM)
		can_timer_high++;
	if (CANGIT & BXOK) { 	/* At least one message pending */
		uint8_t page = CANHPMOB; /* Get the highest priority active mailbox */
		if (page != 0xF0)
			CANPAGE = page;		/* Point to the active mailbox. */
		if (CANSTMOB & RXOK) {
			CAN_rdMsg();			/* Process the message. */
			CANCDMOB = 0x80;		/* Re-enable as a empty Rx buffer. */
		} else if (CANSTMOB & TXOK) {
			tx_full = 0;
			CANCDMOB = 0x00;		/* Disable, empty buffer. */
		}
		CANSTMOB = 0;
	}
}

/*
 * CAN "something happened" interrupt handler
 */
ISR(SIG_CAN_INTERRUPT1)
{
	CAN_dev_poll();
}

/*
 * CAN timer overflow.
 * The internal timer is used for time triggered communication and timestamps.
 * We don't enable this interrupt, as there is little point in a long-term
 * odd-period clock.
 */
ISR(SIG_CAN_OVERFLOW1)
{
	can_timer_high++;
}

void CAN_show_registers(int8_t base)
{
	serprintf(PSTR(" CANGIT %2x.\n"), CANGIT);
	return;
}


/*
 * Local variables:
 *  compile-command: "make at90can.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
