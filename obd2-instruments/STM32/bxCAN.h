/* bxCAN.h: Interface for STM32 CAN controller.  */
#ifndef _BXCAN_H_
#define _BXCAN_H_

enum CAN_ID_Format { Standard_Format=0, Extended_Format=1};
enum CAN_Frame_Type { Data_Frame=0, Remote_Frame=1};

/* A CAN frame in the chip's FIFO.
 * This ignores endian portability rules because it's not portable.
 * It's a device that only exists on a STM chip
 */
struct bxCAN_frame {
	struct {
		uint8_t TxReq:1;		/* Only for Tx FIFO */
		uint8_t RTR:1;          /* CAN read frame or not. */
		uint8_t IDE:1;          /* Identifier type, 11 or 29 bit. */
		union {
			int extid:29;         /* Extended identifier.*/
			struct {		/* Standard identifier.*/
				int pad:18;
				int id:11;
			} std;
		};
	};
	struct {
		int DLC:4;          /* DLC / Data length. */
		int filter:8;		/* FMI Filter match index (Rx only). */
		int timestamp:16;	/* Time stamp of SOF bit on-wire. */
	};
	union {
		uint8_t	data8[8];       /* Frame data, by byte or word. */
		uint16_t	data16[4];
		uint32_t	data32[2];
	};
} bxCANFrame;


/* The hardware-specified filter structure. */
typedef struct {
   /**
    * @brief Filter mode.
    * @note  This bit represent the CAN_FM1R register bit associated to this
    *        filter (0=mask mode, 1=list mode).
    */
   uint32_t                  mode:1;
   /**
    * @brief Filter sclae.
    * @note  This bit represent the CAN_FS1R register bit associated to this
    *        filter (0=16 bits mode, 1=32 bits mode).
    */
   uint32_t                  scale:1;
   /**
    * @brief Filter mode.
    * @note  This bit represent the CAN_FFA1R register bit associated to this
    *        filter, must be set to zero in this version of the driver.
    */
   uint32_t                  assignment:1;
   /**
    * @brief Filter register 1 (identifier).
    */
   uint32_t                  register1;
   /**
    * @brief Filter register 2 (mask/identifier depending on mode=0/1).
    */
   uint32_t                  register2;
} CANFilter;
 
typedef struct  {
  uint32_t id;			/* 11 or 29 bit CAN identifier */
  uint8_t  data[8];		/* Single CAN frame payload data */
  uint8_t  len;			/* Length of data in bytes (CAN DTC) */
  unsigned char  format;	/* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  unsigned char  type;		/* 0 - DATA FRAME, 1 - REMOTE FRAME */
} CAN_msg;

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

extern CAN_msg       CAN_TxMsg;      // CAN messge for sending
extern CAN_msg       CAN_RxMsg;      // CAN message for receiving                                
extern unsigned int  CAN_TxRdy;      // CAN HW ready to transmit a message
extern unsigned int  CAN_RxRdy;      // CAN HW received a message

#endif /* _BXCAN_H_ */
/*
 * Local variables:
 *  compile-command: "make bxCAN.o"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
