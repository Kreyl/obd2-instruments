/* can.h: Prototypes for can-bus.c */

#ifndef _CAN_H
#define _CAN_H

char VIN_str[20];			/* Usually 17 chars, trailing \0. */
extern uint8_t CAN_reset_done;
extern uint8_t CAN_enabled;
extern uint8_t can_verbose;

uint8_t can_reset(void);
uint8_t can_get_status(void);
uint8_t can_setup(void);
uint8_t can_get_pins(void);
void can_set_pins(uint8_t pinval);
void CAN_poll(void);
void CAN_heartbeat(void);
int8_t can_transmit(void);

void can_show_registers(int8_t base);

/* The CAN command buffer.
 * A query uses only 3 bytes to address 0x7DF: 2,mode,pid,0,0,0,0,0
 * A response comes from the module address
 * Both types are always a fixed 8 bytes
 */
struct CAN_command {
	unsigned char cnt;			/* ISO-TP Protocol Control Information. */
	unsigned char mode, pid;
	/* Rework as a union someday, but handle endian conflicts...  */
	unsigned char dataA, dataB, dataC, dataD;
	uint8_t unnamed;			/* Not OBD2 response uses this. */
} can_cmd;
#endif
/*
 * Local variables:
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
