/* can.h: Prototypes for can-bus.c */

#ifndef _CAN_H
#define _CAN_H

extern char VIN_str[20];			/* Usually 17 chars, trailing \0. */
extern uint8_t CAN_reset_done;
extern uint8_t CAN_enabled;
extern uint8_t can_verbose;

int8_t CAN_init(void);
int8_t CAN_start(void);
void CAN_stop(void);
void CAN_poll(void);
void CAN_heartbeat(void);

/* CAN device driver interface. */
uint8_t CAN_dev_init(void);
uint8_t CAN_dev_start(void);
uint8_t CAN_dev_stop(void);
void CAN_dev_poll(void);
int8_t CAN_dev_transmit(void);	/* Returns non-zero on failure */
/* Called from the driver. */
int8_t CAN_process_rx_frame(void);

/* A debugging function that all drivers should implement. */
void CAN_show_registers(int8_t base);

/* Device-specific interface functions.
 * The MCP2515 has pins that can be used as GPIO. */
uint8_t mcp2515_get_pins(void);
void mcp2515_set_pins(uint8_t pinval);
void mcp2515_sleep(uint16_t val);
uint8_t mcp2515_get_status(void);

/* An OBD2 command, sized as a CAN frame.
 * A query uses only 3 bytes to address 0x7DF: 2,mode,pid,0,0,0,0,0
 * A response comes from the module address
 * Both types are always fixed at 8 bytes.
 */
struct CAN_command {
	unsigned char cnt;			/* ISO-TP Protocol Control Information. */
	unsigned char mode, pid;
	/* Use data element names from the standard.
	 * Could rework as a union someday, but handle endian conflicts...  */
	unsigned char dataA, dataB, dataC, dataD;
	/* End of regular OBD2 frame.  Extra byte when no encapsulation. */
	uint8_t unnamed;
	/* End of 8 byte block sent and received. */
#define CAN_SID(std_can_ID) ((std_can_ID)<<5)
	uint16_t id;				/* CAN ID, use CAN_SID() when writing. */
} extern can_cmd;
#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
