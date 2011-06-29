/* bxCAN.c: Interface functions for STM32 CAN controller.  */
/* Device interface functions for the bxCAN controller found on
 * STM32 microcontrollers.
 *
 * Written 2010/2011 by Donald Becker and William Carlson.
 *  This code is written to the Quatum Mechanics interface, originally
 *  developed for the QAR EV controller.  
 * There are a few places in the code rely on the endian mapping.  Since
 *  this controller type only exists only on the STM32, it would be pointless
 *  to write slower or more complex endian-independent code.
 * bxCAN stands for "Basic eXtended Controller Area Network controller".
 *  It's neither basic nor extended, but I didn't make up the name.
 */
#include <armduino.h>
#include <bxCAN.h>
#include "can.h"

/*
 * Reset and initialize the CAN interface, leaving it ready to be started.
 */
uint8_t CAN_dev_init(void)
{
	/* Do a hard reset. */
	APB1RSTR = APB1ENR_CAN1EN;
	APB1RSTR = 0;

	/* Put the bxCAN into sleep mode. */
	CAN1_MCR = CAN_MCR_RESET;

	/* Configure PA11/PA12 as CANRx/Tx. */
	GPIOA_CRH &= (GPIOA_CRH & ~0x000FF000) | 0x000A4000;

	/* Initialize mode, transmit first queued first. */
	CAN1_MCR = CAN_MCR_INRQ;

	/* Set the sample periods for the Bit Timing Register.
	 * Despite the flexibility, there are few valid combinations.
	 * PCLK1 on APB1 = 24 or 36MHz, OBD2 500Kbaud
	 * We choose 8 ticks per bit (8 Time Quanta per Nominal Bit Period).
	 * 1 TQ for propagation segment, 4 TQ for PS1 and 3 TQ for PS2
	 * Pclock divisor: (36MHz / 8TQ) / 500000Kbps = 9 
	 *   (24MHz / 8TQ) / 500000Kbps = 6 
	 */
#if PCLK1 == 36000000
	CAN1_BTR = 0x00240000 | (8 - 1); 	/* Alternative:  0x014B0003; */
#else
	/* Timing for a 24MHz clock. */
	CAN1_BTR = 0x00230000 | (6 - 1); 	/* 8 TQ, 24MHz clock */
#endif

#if defined(CAN_LOOPBACK)
	/* FOR TESTING ONLY: Set loopback mode */
	CAN1_BTR |= CAN_BTR_LBKM;
#endif
	return 0;
}

static int can_startup = 1000000;

/*  CAN initialization.
 * It took experimentation to fill in the missing details of the manual,
 * especially with the Rx filter.
 * Each filter is 8 bytes, and can be configured in one of four ways.
 *  Two 29 bit IDs w/ EID and RTR bits (presumably extended IDs)
 *  One 29 bit ID with a 29 bit mask
 *  Four 11 bit IDs, w/ EID and RTR bits.
 *    The lower 3 bits are ignored
 *  Two 11 bit IDs w/ EID and RTR bits, paired with two 16 bit masks
 */
uint8_t CAN_dev_start(void)
{
	CAN1_MCR |= CAN_MCR_INRQ;

	CAN_FA1R = 0x00000000;	/* Deactivate all filters. */

	/* Set Rx filter.  The reset defaults are used, enabling only filter
	 * 0.
	 * This filter accepts the following standard (non-extended) IDs
	 *  0x420 our arbitrary operating ID for internal messages
	 *  0x7DF the OBD functional (e.g. hailing/broadcast) ID
	 *  0x7e0 this specific controller's OBD ID (possible 0x7e0..7e7).
	 */
	CAN_FM1R = 0x00000003;	/* Filters are exact matches, not ID+mask. */
	CAN_FS1R = 0x00000000;	/* Filters are all 16 bits. */
	CAN_FFA1R = 0x00000001;	/* OBD messages to FIFO1, others to 0  */
	CAN_FILTERS[0] = ((0x7df<<5)<<16) | (0x7e0<<5); /* OBD2 IDs */
	CAN_FILTERS[1] = ((0x7e8<<5)<<16) | (0x420<<5);
#if 0
	/* Redundant, for testing. */
	CAN_FILTERS[2] = ((0x7e0<<5)<<16) | (0x7e0<<5);
	CAN_FILTERS[3] = ((0x7e8<<5)<<16) | (0x7e8<<5);
#endif
	CAN_FA1R = 0x00000001;	/* Activate only filter 0 */
	CAN_FA1R = 0x00000003;	/* Activate filters 0 and 1 */
	CAN_FMR = 0x0E00;		/* Clear filter init bit, now in active mode.*/

	/* Clear INRQ for normal operating mode. */
	CAN1_MCR = CAN_MCR_TXFP | CAN_MCR_ABOM | CAN_MCR_TTCM;

	/* This can hang if there are bus problems or no transceiver. */
	while (CAN1_MSR & CAN_MSR_INAK)
		if (--can_startup < 0)
			return 1;

#if defined(CAN_INTR_MODE)				/* Poll mode only for now. */
	CAN1_IER = (CAN_IER_FMPIE0 | CAN_IER_TMEIE);
	/* Enable interrupts. We use only the Tx0 and Rx0 FIFOs */
	INTR_SETENA(CAN1_Tx_Intr);
	INTR_SETENA(CAN1_Rx0_Intr);
#else
	CAN1_IER = 0;
#endif

	return 0;
}

/* Reset the device, leaving it offline and ready to be initialized. */
uint8_t CAN_dev_stop(void)
{
	/* Disable interrupts. */
	INTR_CLRENA(CAN1_Tx_Intr);
	INTR_CLRENA(CAN1_Rx0_Intr);
	/* Put the bxCAN into sleep mode. */
	CAN1_MCR = CAN_MCR_RESET;
	return 0;
}

/*
 * Transmit a CAN frame in CAN_cmd using buffer BUF_NUM..
 *
 * We are extra clever and scan for an empty Tx buffer with a matching
 * previuosly-written header.  This relies on the chronologic Tx,
 * otherwise it will result in out-of-order multi-frame responses, and
 * possible priority inversion.
 *
 * For simpler controllers the trivial version with fixed ID-slot assignment
 * is better.
 */
int8_t CAN_dev_transmit(void)
{
	volatile uint32_t *tx_fifo;
	/* Still must look for an empty slot, or return 1 if full. */
	if (CAN1_TSR & CAN_TSR_TME0)
		tx_fifo = &CAN1_TI0R;
	else if (CAN1_TSR & CAN_TSR_TME1)
		tx_fifo = &CAN1_TI1R;
	else if (CAN1_TSR & CAN_TSR_TME2)
		tx_fifo = &CAN1_TI2R;
	else
		return 1;

	if (can_verbose)
		serprintf(PSTR("  CAN Tx %3x %2x %2x %2x %2x %2x %2x.\n"),
				  0x7E8, can_cmd.cnt, can_cmd.mode, can_cmd.pid,
				  can_cmd.dataA, can_cmd.dataB, can_cmd.dataC);

	tx_fifo[1] = 8;
	tx_fifo[2] = ((int *)&can_cmd)[0];
	tx_fifo[3] = ((int *)&can_cmd)[1];
	tx_fifo[0] = (can_cmd.id << 21) | 1;
	return 0;
}

/*
 * Read a CAN frame
 */
void CAN_rdMsg(volatile uint32_t *rx_fifo)
{
	((int *)&can_cmd)[0] = rx_fifo[2];
	((int *)&can_cmd)[1] = rx_fifo[3];
	if (can_verbose)
		serprintf(PSTR(" CAN Rx %3x %2x %2x %2x.\n"),
				  rx_fifo[0] >> 21, can_cmd.cnt,
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
	if (CAN1_RF0R & CAN_RxFIFO_FMP) { 	/* At least one message pending */
		CAN_rdMsg(&CAN1_RI0R);			/* Process the message. */
		CAN1_RF0R |= CAN_RxFIFO_Release; /* Release FIFO 0 mailbox */

	}
 	/* Message pending FIFO 1, same actions */
	if (CAN1_RF1R & CAN_RxFIFO_FMP) {
		CAN_rdMsg(&CAN1_RI1R);
		CAN1_RF1R |= CAN_RxFIFO_Release;
	}
	if (CAN1_TSR & CAN_TSR_RQCP0) {
		CAN1_TSR |= CAN_TSR_RQCP0;
	}
}

/*
 * CAN transmit done interrupt handler
 */
ISR(CAN1_Tx)
{
	if (CAN1_RF0R & CAN_RxFIFO_FMP) { 	/* At least one message pending */
		CAN_rdMsg(&CAN1_RI0R);			/* Process the message. */
		/* Release FIFO 0 mailbox */
		CAN1_RF0R |= CAN_RxFIFO_Release;
	}
	if (CAN1_RF1R & CAN_RxFIFO_FMP) { 	/* Message pending FIFO 1*/
		CAN_rdMsg(&CAN1_RI1R);
		/* Release FIFO 0 mailbox */
		CAN1_RF1R |= CAN_RxFIFO_Release;
	}
	if (CAN1_TSR & CAN_TSR_RQCP0) {
		CAN1_TSR |= CAN_TSR_RQCP0;
		CAN1_IER &= ~CAN_IER_TMEIE;                   // disable  TME interrupt
	}
}

/*
 * CAN receive interrupt handler
 */
ISR(CAN1_Rx0)
{
	if (CAN1_RF0R & CAN_RxFIFO_FMP) { 	/* At least one message pending */
		CAN_rdMsg(&CAN1_RI0R);
	}
}

void CAN_show_registers(int8_t base)
{
	/* We should be lazier, which would avoid I/O function dependencies. */
	serprintf("bxCAN control %4x status %4x Tx status %4x Rx %4x %4x\n"
			  "bxCAN errors %4x\n",
			  CAN1_MCR, CAN1_MSR, CAN1_TSR, CAN1_RF0R, CAN1_RF1R,
			  CAN1_ESR);
	return;
}


/*
 * Local variables:
 *  compile-command: "make bxCAN.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
