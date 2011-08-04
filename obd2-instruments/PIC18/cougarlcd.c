/* cougarlcd.c: Translation of the Cougar LCD code from ASM to C. */
/*
  Written 2011 by Donald Becker and Hugo William Becker, based on
  ASM code written by Chris Freyman.

  This file contains a translation from 'cougarlcd_v1_4b.ASM' to C.
  We wrote this to understand and support the hardware platform with
  our OBD2-instruments system.  A independently written source file
  implements similar functionality with the addition of CAN support.

  The build environment is the SDCC compiler, with its library and
  include files.
  See the bottom of the file for the basic compiler command line.
  The processor family and part number is set on the command line.
  Note that __naked functions in SDCC have no prologue or postlogue
  thus need an explict __asm return __endasm;
*/

#include <stdint.h>
#include "pic18fregs.h"

/* I/O pin assignments. */
#define SW1 (PORTBbits.RB0_PORTB)
#define LCD (PORTB)				/* Only upper nibble connected. */
#define SW2 (PORTAbits.RA4)
#define RLED  (PORTCbits.RC0)
#define GLED  (PORTCbits.RC1)
#define ALRM  (PORTCbits.RC3)
#define RS PORTCbits.RC4		/* LCD Register Select (Instruction/Data). */
#define E  PORTCbits.RC5		/* LCD Enable pin, latched on high->low */

#define F_CPU 8000000			/* CPU oscillator frequency. */

#if defined SDCC
/* The SDCC and PIC compiler-fu.
 * This magic communicates the processor "fuse" configuring settings to the
 * programmer.  This info is used to configure pre-execution settings such
 * as the hardware clock source.
 */

code char at __CONFIG1H conf1 = _OSC_HS_1H & _FCMEN_OFF_1H & _IESO_OFF_1H;
code char at __CONFIG2L conf2l = _PWRT_ON_2L & _BOREN_OFF_2L & _BORV_2_2L;
code char at __CONFIG2H conf2h = _WDT_OFF_2H & _WDTPS_1024_2H;
code char at __CONFIG3H conf3h =
	_MCLRE_ON_3H & _LPT1OSC_OFF_3H & _PBADEN_OFF_3H;
code char at __CONFIG4L conf4l =
	_DEBUG_OFF_4L & _XINST_OFF_4L & _LVP_OFF_4L & _STVREN_OFF_4L;
code char at __CONFIG5L conf5l = _CP0_OFF_5L & _CP1_OFF_5L;
code char at __CONFIG5H conf5h = _CPB_OFF_5H & _CPD_OFF_5H;
/* Check this. */
code char at __CONFIG6L conf6l = 0xFF; /* ToDo:verify */
code char at __CONFIG6H conf6h = 0xFF; /* ToDo:verify */
code char at __CONFIG7L conf7l = _EBTR0_OFF_7L & _EBTR1_OFF_7L;

#pragma std_sdcc99 
#pragma stack 0x200 64

/* Instructions that should be in a header file. */
#define swapf(nibblepair)  { __asm SWAPF _##nibblepair , W __endasm;}
#define naked_return  __asm RETURN __endasm
#else
#warning Unknown compiler. PIC18 configuration space initialization omitted.
#endif

typedef union uint24_un {
	struct {
		uint8_t L, M, H;
	};
	struct {
		uint8_t High;
		uint16_t ML;
	};
} uint24_t;

/* Constants used at run time. */
static const uint8_t MT_Max= 200; /* max motor temp for error LED */
static const uint8_t CT_Max= 167; /* max controller temp for error LED */
static const uint8_t BT_Max= 125; /* max battery temp for error LED */
/* Traction battery capacity in Amp/seconds (only high/mid byte of 24-bit) */
static const uint8_t bcapacityM = 0x5E;
static const uint8_t bcapacityH = 0x06;
static const uint8_t pmile = 32;
static const char PWM_Max= 0xFF;
static const char Sysval = 0x80; /* ToDo: configure as bitmap */
static const uint8_t ALRM_lim = 21; /* Percentage SOC where alarm is tripped */
/* Length of Profile 1 message that establishes offset to access Profile 2 */
static const char PF2_Offset = 26;

/* Adjustment for pack voltage.  This also corrects for minor gain
 * transfer error in HCNR201. Value=correction x 10 */
/* Adjustment for accessory voltage: compensate for diode drop of D1.
 * Value=correction x 100 */
const int8_t Vpack_trim = 0, Vacc_trim = 70;
/* Scale for motor amps and battery amps from serial message.
 * Value/16= scale. Value of 16 is 1:1, 20 is 1.25:1,etc */
static const char sAmp_scale = 16;
/* Battery amp charge efficiency. 16=100%, 15=94%, 14=87.5%, 13=81%, 12=75% */
static const char cEfficiency = 15;
/* Charge amp level where cTime stops incrementing & mode LED stops blinking */
static const char min_cAmp = 1;
 /* Update rate for displayed information.
  * 1= same as old software. 50=updates once per second */
static const uint8_t D_refresh = 1;


/* Calculate the moving average.
 * Input passed in tempH/L*/

static void INIT_LCD(void);
static void SECOND_LINE(void);
static void COMMAND(uint8_t cmd);
static void COMMAND8(uint8_t command) __wparam;
static void DISPLAY(char c)  __naked;
static void A2DLoop(void);
static void A2DAVG(void);
static void Hysteresis(void);
static void RXconvrt(void);
static void mAmp_stat(void);
static void MDELAY(uint8_t microseconds) __wparam;
static void MS_DELAY(char tens_of_milliseconds) __wparam;
static void Hex2BCD(void);
static void BCD2Hex(void);
static void Fracmult(uint8_t fraction4_4)  __wparam;/* Call-return in Arg2 */
static char GetChar(void);
static char GetTemp(void);
static char GetTX(void);
static void SOC_Calc(void);
static void Range(void);
static uint8_t CapTempAdj(void);
static char Zerosupress(void);
static void Button(void);
static void TX_Mode(void);
static uint8_t F2C(uint8_t fahrenheit) __wparam;
static void Capstore(void);
static void EEsave(void);
static uint8_t EERead(uint8_t index);

/* Calculate the moving average, using AVG_FACTOR as the decay period. */
static uint16_t ADC_average(uint16_t running_avg, uint8_t avg_factor);

/* Commands for HD44780 LCD controllers. */
enum LCD_commands {
	CLRSCR=1, HOME=2,
	OFF=0x08, ON=0x0C,		/* Underline and blink off */
	LEFT=0x18, RIGHT=0x1C,	/* Display shift  0x10/14 for cursor move */
	LCD_4bit=0x20,			/* Use 4 bit interface */
	LIN2=0xC0
};

/* cblock  0x00 */
/* Locations for saving context during interrupts. */
static uint8_t W_TEMP, STATUS_TEMP, BSR_TEMP, Off_temp, Off_tempH;
static uint16_t Ptr_temp, PROD_temp, Arg2_temp, temp;
static int16_t avg;

/* cblock  0x1A */
char CNT, CNT1, CNT2, CNT3, CNT4, CNT_Temp;
uint16_t Num;
char Dig4, Dig3, Dig2, Dig1;
char Digtemp;
uint8_t Offset, OffsetH;		/* Typically used as 8 bits. */
uint16_t Temp_ptr;
uint8_t Messnum;					/* Message number */
uint16_t mAmp, bAmp, bcAmp, avolt;
uint8_t capacityH;
uint16_t capacity;
uint8_t SOC, SOC_temp;
char Miles;
uint8_t FG_Scale, bTemp;
uint16_t cTemp;
uint8_t mTemp;
uint16_t Volt;
char Dec1, Dec2, Dec3;
char CHAR1, CHAR2, CHAR3, CHAR4, CHAR5, CHAR6, CHAR7, CHAR8, CHAR9;
uint16_t Hex, Arg2;
char RES0, RES1, RES2, RES3;
int8_t captempH;
uint16_t captemp, bAmptemp;
int8_t LCDtemp, RX_temp, RX_count, EE_count;
union Sys_Config_type {				/* System status flags */
	struct {
		unsigned DisplayMode: 1; /*  */
		unsigned b1: 1; /*  */
		unsigned bAmpSource:  1; /* Battery amp source.
								  * 0=local A/D , 1=motor controller */
		unsigned UseMetric 	: 1;	 /* Display metric units */
		unsigned PeukertBypass: 1;	 /* Peukert bypass (1.0) */
		unsigned Peukert1_20: 1;	 /* Peukert exponent 1.20 */
		unsigned bAmp300mVOffset: 1;	/* Do battery amp 300mV correction */
		unsigned b7   	: 1;	 /*  */
	};
	uint8_t byte;				/* Clear or set as whole byte. */
} Sys_Config;
uint16_t pdiv, pct20, Vavg, aVavg, Bavg;
uint8_t Disp_count, Avg_div;
uint8_t Captempcomp;

union LZstat_type {				/* System status flags */
	struct {
		unsigned LeadingZero: 1; /* Supress leading zeros */
		unsigned ProfileMode: 1; /* Profile mode */
		unsigned ChargeMode	: 1; /* Charge mode */
		unsigned NegTemp   	: 1; /* Negative temperature */
		unsigned Overtemp  	: 1; /* Overtemperature */
		unsigned mtrAmpsLow	: 1; /* Motor current is negligible */
		unsigned TermCount 	: 1; /* Terminal count */
		unsigned CommErr   	: 1; /* Communications error. */
	};
	uint8_t byte;				/* Clear or set as whole byte. */
} LZstat;
uint8_t C_hours, C_minutes, CM_Delay, RF_Count;


/* High priority interrupt service routine used for serial port receive
 * from controller.  This is hard-coded to recognize the Cougar "real time
 * display" format, and fragile because if it.
 */
static void HIGH_ISR(void) __interrupt(1)
{
	/* Save the W/working, status and bank select registers */
	PIR1bits.RCIF = 0;
	if (RX_count == 0)
		LZstat.CommErr = 0;
	RX_count++;
	if (RCSTAbits.FERR)
		LZstat.CommErr = 1;
	RX_temp = RCREG;
	if (RX_temp != 0x0A) {
		switch(RX_count) {
		case 18: CHAR3 = RX_temp; break;
		case 19: CHAR2 = RX_temp; break;
		case 20: CHAR1 = RX_temp; break;
		case 33: CHAR6 = RX_temp; break;
		case 34: CHAR5 = RX_temp; break;
		case 35: CHAR4 = RX_temp; break;
		case 54: CHAR9 = RX_temp; break;
		case 55: CHAR8 = RX_temp; break;
		case 56: CHAR7 = RX_temp; break;
		}
	} else {
		if (RX_count == 67)
			LZstat.CommErr = 1;
		RX_count = 0;
	}
#if 0
	{
		if (RX_count == 18) {
			CHAR3 = RX_temp;
			return;
		}
		if (RX_count == 19) {
			CHAR2 = RX_temp;
			return;
		}
		if (RX_count == 20) {
			CHAR1 = RX_temp;
			return;
		}
		if (RX_count == 33) {
			CHAR6 = RX_temp;
		}
		if (RX_count == 34) {
			CHAR5 = RX_temp;
		}
	}
#endif
	/* ToDo: Restore the W/working, status and bank select registers */
	return;
}

/* The "low priority" interrupt is used for the 1Hz timer. */
static void LOW_ISR() __interrupt(2)
{
	/* Save the W/working, status and bank select registers */
#if 0
	W_TEMP = working;
#endif
	STATUS_TEMP = STATUS;
	BSR_TEMP = BSR;
	/* Save other variables. */
	Off_temp = Offset;
	Off_tempH = OffsetH;
	PROD_temp = PROD;
	CNT_Temp = CNT;
	Arg2_temp = Arg2;

	T0CONbits.TMR0ON = 0;		/* Disable timer0 */
	INTCONbits.TMR0IF = 0;		/* Clear timer0 interrupt flag */

	if (CM_Delay) CM_Delay--;
	if ( ! LZstat.ChargeMode) {	/* Driving/discharge mode. */
		if (bAmp <= 1) {		/* If nearly discharged skip SoC adjust */
			bAmptemp = bAmp;
			bAmptemp >>= 3;
			/* At low reserve capacity we don't do Peukert adjustment. */
			if (bAmptemp == 0) {
				RES0 = bAmp;
				RES1 = 0;
			} else {
				if (bAmptemp >= 48)
					bAmptemp = 48;
				Offset = bAmptemp;
#if 0
				if (Sys_Config.Peukert1_20)
					if (Sys_Config.PeukertBypass)
						;
				/* ToDo: fill in logic for Peukert table selection. */
				/* ToDo: line 427 */
#endif
				Offset += 85;
				Arg2 = bAmp;
				Fracmult(GetChar());
			}
			capacity -= RES0 + RES1<<8;
			EE_count++;
			/* Backup capacity to EEPROM every minute. */
			if (EE_count >= 60)
				Capstore();
		}
		/* Alarm */
		if (SOC < ALRM_lim) {
			ALRM ^= 1;
			RLED ^= 1;
		}
		/* mAmp_test */
		if (LZstat.mtrAmpsLow == 0 ||
			LZstat.TermCount == 0 ||
			Disp_count < 6) {
			Disp_count++;
			if (Disp_count == 5)
				LZstat.TermCount = 1;
		}
	} else {					/* Charging mode */
		if (bAmp > min_cAmp) {
			GLED ^= 1;
			if (++EE_count > 59) {
				Capstore();
				C_minutes++;
				if (C_minutes == 60) {
					C_hours++;
					C_minutes = 0;
				}
			}
			if (SOC >= 100) {
				Arg2 = bAmp;
				Fracmult(cEfficiency);
				capacity += RES0;			/* Todo: Carry into capacityH.  check RES0 result return */
			}
		}
	}

	/* Set timer period to 34285 = 0x85ED. */
	TMR0L = 0xED;
	TMR0H = 0x85;

	/* Restore the W/working, status and bank select registers */
#if 0  /* ToDo: */
	working = W_TEMP;
#endif
	STATUS = STATUS_TEMP;
	BSR = BSR_TEMP;
	/* Restore other variables.
	 * These could have been only selectively saved. */
	Offset = Off_temp;
	OffsetH = Off_tempH;
	PROD = PROD_temp;
	CNT = CNT_Temp;
	Arg2 = Arg2_temp;
	T0CONbits.TMR0ON = 1;		/* Enable timer0 */

	return;
}

/* START AKA main(). */
static void main(void) __interrupt(0) __naked
{
	/* Initialize ports. */
	/* Select the external OSC1 8MHz crystal. */
	OSCCON = 0x74;
	PORTC = 0;
	PORTA = 0;
	PORTB = 0;
	SSPCON1bits.SSPEN = 0;		/* Disable serial port */
	TRISB = 0x0F;				/* Upper 4 bits LCD control.  B0 pushbutton */
	TRISC = 0x80;				/* Serial port (B7,B6) LED drive, LCD select */
	TRISA = 0x3F;				/* Analog and pushbutton inputs */
	/* Set the ADC to Vdd/Vss reference, AN0..AN5 as anal og inputs. */
	ADCON1 = 0x0A;
	/* Initialize the USART */
	BAUDCON = 0;				/* Async 8 bit mode. */
	SPBRG = 0x19;
	TXSTA = 0xA4;
	RCSTA = 0x10;				/* Leave reset. */
	PIR2 = PIR1 = 0;
	PIE1 = 0x20;
	PIE2 = 0;
	/* Setup interrupts. */
	INTCON = 0xE0;				/* Enable intrs: peripherals and timer0 */
	RCONbits.IPEN = 1;			/* Set interrupt priority mode */
	IPR1bits.TXBIP = 1;			/* Set UART as a high priority interrupt. */
	INTCON2bits.TMR0IP = 0;		/* Set timer0 as a low priority interrupt. */
	MS_DELAY(20);				/* 200msec wait.  Power-up delay for LCD?  */
	INIT_LCD();
	COMMAND(CLRSCR);
	MS_DELAY(20);				/* Another 200msec wait */
	/* Clear all variables, painfully one-by-one. */
	Messnum = RX_count = Disp_count = mAmp = cTemp =
		bcAmp = LZstat.byte = C_hours = C_minutes = RF_Count = 0;
	CHAR1 = CHAR2 = CHAR3 = CHAR4 = CHAR6 = CHAR7 = CHAR8 = CHAR9 = 0;
	CHAR5 = 6;
	Sys_Config.byte = Sysval;
	CM_Delay = 2;				/* Set charge mode enable delay timer */
	/* Fuel Gauge PWM output */
	TRISCbits.TRISC2 = 1;
	PR2 = 0xFF;
	CCP1CON = 0x2C;
	CCPR1L = PWM_Max;
	T2CON = 6;
	TRISCbits.TRISC2 = 0;
	/* Setup timer. */
	T0CON = 5;
	TMR0L = 0xED;
	TMR0H = 0x85;
	EE_count = 0;
	/* Test for SoC reset request: switch 1 pressure during reset . */
	if ( ! SW1) {
		/* Read in stored capacity */
		capacityH = EERead(1);
		capacity = EERead(2)<<8;
	} else {
		char c;
		capacityH = bcapacityH;
		capacity = bcapacityM<<8;
		Capstore();
		Offset = 54;			/* " RESETTING SOC " message */
		while ((c = GetChar()) != 0) {
			DISPLAY(c);
			Offset++;
		}
	}
	CNT1 = CNT = 0;
	captempH = bcapacityH;
	captemp = bcapacityM<<8;		/* Should be a 24bit copy w/ low byte 0 */
	while ((captemp -= 100) > 0)
		CNT++;
	pdiv = CNT;
	/* Missing code. */

	ADCON0 = 0x11;				/* ADC4, battery temperature */
	ADCON0bits.GO_DONE = 1;
	while (! ADCON0bits.GO_DONE)
		;
	Captempcomp = ADRESH;
	RCSTAbits.SPEN = 1;			/* Enable serial receive. */
	WDTCONbits.SWDTEN = 1;		/* Enable watchdog timer. */

	/* Main operation loop. */
	do {
		ClrWdt();
		MS_DELAY(1);
		if (RCSTAbits.OERR) {
			RCSTAbits.CREN = 0;		/* Restart to clear overflow. */
			LZstat.CommErr = 1;
			RCSTAbits.CREN = 1;		/* Restart to clear overflow. */
		}
		if (RX_count == 0)
			RXconvrt();
		A2DLoop();
		SOC_Calc();
		Temp_ptr = mTemp;
		Temp_ptr == 20;
		mTemp = GetTemp();
		bTemp = GetTemp();
		if (SW1)
			Button();
		if (SW2)
			TX_Mode();
		RLED = 0;
		if (mTemp > MT_Max || bTemp > BT_Max || cTemp > CT_Max) {
			RLED = 1;
			/* Set overtemp status */
			if (Sys_Config.DisplayMode && ! LZstat.Overtemp) {
				LZstat.Overtemp = 1;
			}
			Offset = 36;		/* "Tmot Tcont Tbat " message */
			Messnum = 2;		/* Message number. */
		} else
			LZstat.Overtemp = 0;

		/* Display routine. */
		if (++RF_Count > D_refresh) {
			RF_Count = 0;
			OffsetH = Offset;	/* Save for later */
			if ( ! LZstat.ChargeMode) {
				char c;
				/* Line 1 */
				while ((c = GetChar()) != 0) {
					if (Sys_Config.UseMetric) {
						Digtemp = c;
						if (c == 'M')
							c = 'K';
					}
					DISPLAY(c);
					Offset++;
				}
				/* Line 2 */
				Offset = OffsetH;	/* Saved earlier. */
				SECOND_LINE();
#if 0
				if (Messnum == 2) {
				}
#endif
			} else {			/* Charge mode */
				char c;
				Offset = 70;
				/* Line C1 */
				while ((c = GetChar()) != 0) {
					DISPLAY(c);
					Offset++;
				}
				/* Line C2 */
				Offset = OffsetH;	/* Was this really saved earlier? */
				SECOND_LINE();
				Num = C_hours;
				Hex2BCD();		/* Misnamed, really binary to ASCII */
				DISPLAY(Dig2);
				DISPLAY(Dig1);
				DISPLAY(':');
				Num = C_minutes;
				Hex2BCD();
				DISPLAY(Dig2);
				DISPLAY(Dig1);
				DISPLAY(' ');
				DISPLAY(' ');

				Num = bAmp;
				Hex2BCD();
				LZstat.LeadingZero = 0;
				Digtemp = Dig2;
				DISPLAY(Zerosupress());
				DISPLAY(Dig1);
				DISPLAY(' ');
				DISPLAY(' ');

				Num = SOC;
				Hex2BCD();
				LZstat.LeadingZero = 0;
				Digtemp = Dig3;
				DISPLAY(Zerosupress());
				Digtemp = Dig2;
				DISPLAY(Zerosupress());
				DISPLAY(Dig1);
				DISPLAY('%');
			}
		}
	} while(1);

	/* No exit */
}


/* Note: Either the ASM code has the offsets wrong, or there is
 * silent byte padding. */
char Text_Table[] =
	"Volts mAmp bAmp \0\0"
	"aVolt  SOC  MTE \0\0"
	"Tmot Tcont Tbat \0\0"
	" RESETTING SOC \0"
	"cTime Amps SOC \0";

#if defined(Sysval) && Sysval >= 128

uint8_t Peukert_1_30_tbl[] = {			/* Peukert 1.30 multipliers */
	0x12,0x16,0x19,0x1B, 0x1D,0x1E,0x20,0x21,
	0x22,0x23,0x24,0x24, 0x26,0x27,0x28,0x29, /* index 64 */
	0x29,0x2A,0x2B,0x2B, 0x2C,0x2D,0x2D,0x2E,
	0x2E,0x2F,0x30,0x30, 0x31,0x31,0x32,0x32, /* index 80 */
	0x33,0x33,0x34,0x34, 0x34,0x35,0x35,0x35,
	0x36,0x36,0x36,0x37, 0x37,0x37,0x38,0x38, /* index 96 */
};
uint8_t Peukert_1_25_tbl[] = {			/* Peukert 1.25 multipliers */
	0x12,0x15,0x17,0x19, 0x1A,0x1B,0x1C,0x1D,
	0x1E,0x1F,0x20,0x20, 0x21,0x22,0x22,0x23,	/* index 112 */
	0x23,0x24,0x24,0x25, 0x25,0x26,0x26,0x26,
	0x27,0x27,0x28,0x28, 0x28,0x29,0x29,0x29,	/* index 128 */
	0x2A,0x2A,0x2A,0x2B, 0x2B,0x2B,0x2B,0x2C,
	0x2C,0x2C,0x2C,0x2D, 0x2D,0x2D,0x2D,0x2D,	/* index 144 */
};
uint8_t Peukert_1_20_tbl[] = {			/* Peukert 1.20 multipliers */
	0x12,0x14,0x15,0x17, 0x18,0x19,0x1A,0x1A,
	0x1B,0x1B,0x1C,0x1C, 0x1D,0x1D,0x1E,0x1E,	/* index 160 */
	0x1E,0x1F,0x1F,0x1F, 0x1F,0x20,0x20,0x20,
	0x20,0x21,0x21,0x21, 0x21,0x22,0x22,0x22,	/* index 176 */
	0x22,0x23,0x23,0x23, 0x23,0x23,0x24,0x24,
	0x24,0x24,0x24,0x25, 0x25,0x25,0x25,0x25,	/* index 192 */
};
#else
uint8_t Peukert_1_15_tbl[] = {
	0x11,0x13,0x14,0x15,0x15,0x16,0x17,0x17,
	0x17,0x18,0x18,0x18,0x19,0x19,0x19,0x19,
	0x1A,0x1A,0x1A,0x1A,0x1B,0x1B,0x1B,0x1B,
	0x1B,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1D,
	0x1D,0x1D,0x1D,0x1D,0x1D,0x1D,0x1D,0x1E,
	0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E
};
uint8_t Peukert_1_10_tbl[] = {
	0x11,0x12,0x12,0x13,0x13,0x14,0x14,0x14,
	0x15,0x15,0x15,0x15,0x15,0x16,0x16,0x16,
	0x16,0x16,0x16,0x16,0x16,0x17,0x17,0x17,
	0x17,0x17,0x17,0x17,0x17,0x17,0x17,0x17,
	0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
	0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18
};
uint8_t Peukert_1_05_tbl[] = {
	0x10,0x11,0x11,0x11,0x12,0x12,0x12,0x12,
	0x12,0x12,0x12,0x12,0x12,0x12,0x12,0x13,
	0x13,0x13,0x13,0x13,0x13,0x13,0x13,0x13,
	0x13,0x13,0x13,0x13,0x13,0x13,0x13,0x13,
	0x13,0x13,0x13,0x14,0x14,0x14,0x14,0x14,
	0x14,0x14,0x14,0x14,0x14,0x14,0x14,0x14,
 };
#endif

uint8_t Temp_Table[45*8+1] = {
	0x05,0x07,0x09,0x0B,0x0D,0x0F,0x10,0x12,
	0x14,0x16,0x17,0x19,0x1A,0x1C,0x1D,0x1E,
	0x20,0x21,0x23,0x24,0x25,0x27,0x28,0x29,
	0x2A,0x2C,0x2D,0x2E,0x2F,0x30,0x31,0x32,
	0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,
	0x3C,0x3D,0x3E,0x3F,0x40,0x41,0x42,0x43,
	0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,
	0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,
	0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,
	0x5C,0x5D,0x5E,0x5F,0x60,0x61,0x62,0x63,
	0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x6B,
	0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,
	0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x7B,
	0x7C,0x7D,0x7E,0x7F,0x81,0x82,0x83,0x84,
	0x85,0x86,0x87,0x89,0x8A,0x8B,0x8C,0x8D,
	0x8E,0x90,0x91,0x92,0x93,0x95,0x96,0x97,
	0x99,0x9A,0x9C,0x9D,0x9E,0xA0,0xA1,0xA3,
	0xA4,0xA6,0xA7,0xA9,0xAA,0xAC,0xAE,0xAF,
	0xB1,0xB3,0xB5,0xB6,0xB8,0xBA,0xBC,0xBE,
	0xC0,0xC2,0xC5,0xC7,0xC9,0xCB,0xCE,0xD0,
	0xD3,0xD6,0xD9,0xDB,0xDE,0xE2,0xE5,0xE8,
	0xEC,0xEF,0xF3,0xF7,0xFB,0xFF,0xFF,0xFF,
	0x05,0x07,0x09,0x0B,0x0D,0x0F,0x10,0x13,
	0x14,0x16,0x17,0x19,0x1A,0x1C,0x1D,0x1E,
	0x20,0x21,0x22,0x24,0x25,0x26,0x27,0x28,
	0x29,0x2A,0x2C,0x2D,0x2E,0x2F,0x30,0x31,
	0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
	0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x40,0x41,
	0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,
	0x48,0x49,0x4A,0x4B,0x4C,0x4C,0x4D,0x4E,
	0x4F,0x50,0x51,0x52,0x52,0x53,0x54,0x55,
	0x55,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,
	0x5C,0x5D,0x5E,0x5F,0x5F,0x60,0x61,0x62,
	0x63,0x64,0x65,0x65,0x66,0x67,0x68,0x69,
	0x6A,0x6B,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,
	0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,
	0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F,
	0x80,0x81,0x81,0x82,0x84,0x85,0x86,0x87,
	0x88,0x89,0x8A,0x8B,0x8C,0x8D,0x8E,0x90,
	0x91,0x92,0x93,0x94,0x96,0x97,0x98,0x9A,
	0x9B,0x9C,0x9D,0x9F,0xA0,0xA2,0xA3,0xA5,
	0xA6,0xA8,0xAA,0xAB,0xAD,0xAF,0xB0,0xB2,
	0xB4,0xB6,0xB8,0xBA,0xBC,0xBE,0xC0,0xC3,
	0xC5,0xC7,0xCA,0xCC,0xCF,0xD2,0xD5,0xD8,
	0xDB,0xDF,0xE2,0xE6,0xEB,0xEF,0xF4,0xF9,
	0xFE
};

char TX_Table[] = "rtd-period \0"
"200\0"
"c-rr 15\r"
"bat-amps-lim 350\r\0"
"c-rr 5\r"
"bat-amps-lim 190\r";


/* Omitted: peukert multiplier tables. */

static void INIT_LCD(void)
{
	/* Reset three times. */
	COMMAND8(0x30);
	MS_DELAY(10);
	COMMAND8(0x30);
	MS_DELAY(10);
	COMMAND8(0x30);
	MS_DELAY(10);

	COMMAND8(0x28);					/* Switch the LCD to 4 bit mode */
	COMMAND(0x28);					/* In 4 bit mode, configure for two lines */
	COMMAND(ON);
	COMMAND(0x06);
	return;
}

static void SECOND_LINE(void)
{
	COMMAND(LIN2);
	return;
}

/* Executes a command in the W reg to the LCD Module.
 * Precondition: module set to 4 bit mode.
 */
static void COMMAND(uint8_t cmd)
{
	RS = 0;		 /* Instruction/data select = 0 */
	LCDtemp = cmd;
	LCD = cmd & 0xF0;
	E = 1;						/* Set LCD enable */
	MDELAY(50);
	INTCONbits.GIE = 0;			/* Disable interrupts globally. */
	E = 0;
	MDELAY(50);
	swapf(LCDtemp);
	LCD = LCDtemp & 0xF0;  
	E = 1;						/* Set LCD enable */
	MDELAY(50);
	E = 0;
	INTCONbits.GIE = 1;			/* Re-enable interrupts. */
	/* Delay 500 usecs. The max delay per call is 255 usec.  */
	MDELAY(250);
	MDELAY(250);
	RS = 1;						/* Instruction/data select = 1 */
	return;
}

/* Send COMMAND to the LCD Module.
 * This is used briefly to switch into 4 bit mode.
 */
static void COMMAND8(uint8_t command)  __wparam
{
	RS = 0;		 /* Instruction/data select = 0 */
	LCD = command;
	E = 1;						/* Clock LCD enable */
	MDELAY(250);
	E = 0;
	MDELAY(250);
	MDELAY(250);
	RS = 1;						/* Instruction/data select = 1 */
}

static void DISPLAY(char c) __naked  __wparam
{
	RS = 1;						/* Instruction/data select to data. */
	LCDtemp = c;
	LCD = c & 0xF0;
	E = 1;						/* Clock LCD enable */
	MDELAY(40);
	INTCONbits.GIE = 0;			/* Disable interrupts globally. */
	E = 0;
	MDELAY(40);
	swapf(LCDtemp);
	LCD = LCDtemp  & 0xF0;  
	E = 1;						/* Clock in second nibble. */
	MDELAY(40);
	E = 0;
	INTCONbits.GIE = 1;			/* Re-enable interrupts. */
	/* Delay 500 usecs. The max delay per call is 255 usec.  */
	MDELAY(250);
	MDELAY(250);

	naked_return;				/* Should be implemented by the compiler */
}

/* Read the A/D channels and store the scaled results. */
static void A2DLoop(void)
{
	/* Set ADC right justified, 20 Tad conversion time, Fosc/32. */
	ADCON2 = 0xBA;
	ADCON0 = 1;
	temp = Vavg;
	Avg_div = 3;
	A2DAVG();
	Vavg = temp;
	if (Vpack_trim >= 0)
		avg += Vpack_trim;
	else if ((avg -= Vpack_trim) < 0)
			avg = 0;
	/* Hysteresis routine. */
	temp = Volt;
	Hysteresis();	/* '0' passed in W, but never used. */
	Volt = avg;
	/* Use the data from the motor controller if set. */
	if (Sys_Config.bAmpSource) {
		bAmp = bcAmp;
	} else {
		ADCON0 = 5;
		temp = Bavg;
		Avg_div = 5;
		A2DAVG();
		Bavg = temp;
		if (C_hours == 0) {
			if (C_minutes <= 1)
				LZstat.ChargeMode = 0;
			/* Clear temp compensation of SoC */
			if (C_minutes < 1)
				Sys_Config.bAmp300mVOffset = 0;
		}
		if ((avg -= 30) < 0) {					/* 300mV offset */
			/* Enter charge mode. */
			avg = -avg;
			/* If charging current is less than 3 amps, ignore. */
			if (avg > 2) {
				if ( ! CM_Delay)
					LZstat.ChargeMode = 1;
			} else
				avg = 0;
		}
		if (avg >= 2) {
			/* Hysteresis routine. */
			temp = bAmp;
			Hysteresis();	/* '2' passed in W, but never used. */
			bAmp = avg;
		}
#if defined(USE_LEM_HASS)
		Arg2 = bAmp;
		bAmp = Fracmult(29);
#endif
	}

	/* ADC channel 2, accessory voltage. */
	ADCON0 = 0x09;
	temp = aVavg;
	Avg_div = 3;
	A2DAVG();
	aVavg = temp;

	avg += Vacc_trim;
	temp = avolt;
	Hysteresis();
	avolt = avg;

	/* Set ADC left justified, 20 Tad conversion time, Fosc/32. */
	ADCON2 = 0x3A;
	/* ADC Channel 3, motor temp */
	ADCON0 = 0x0D;
	ADCON0bits.GO_DONE = 1;
	while (! ADCON0bits.GO_DONE)
		;
	temp = mTemp;
	avg = ADRESH;
	Hysteresis();
	mTemp = avg;

	/* ADC Channel 4, battery temp */
	ADCON0 = 0x11;
	ADCON0bits.GO_DONE = 1;
	while (! ADCON0bits.GO_DONE)
		;
	temp = bTemp;
	avg = ADRESH;
	Hysteresis();
	bTemp = (uint8_t)avg;

	/* ADC Channel 10, FG_Scale */
	ADCON0 = 0x29;
	ADCON0bits.GO_DONE = 1;
	while (! ADCON0bits.GO_DONE)
		;
	FG_Scale = ADRESH;
	if ((int)FG_Scale > 202)			/* Limit to 202 */
		FG_Scale = 202;
	return;
}

/* Calculate moving average, AVG= Old AVG-(Old Avg/16)+ new sample
 * Return sample sum in tempL/H and new average in avgL/H.
 */
static void A2DAVG(void)
{
	ADCON0bits.GO_DONE = 1;
	while (! ADCON0bits.GO_DONE)
		;
	temp = avg - (temp>>4) + ADRESH;
}

/* Hysteresis for value in TEMP and AVG, based on DEADBAND.
 * Implemented in negative (decreasing) direction only.
 * Result return in AVG 
 * Bug: Some code assumes that DEADBAND is passed in W, but it is
 * really a hard-coded constant.
 */
static int16_t RES23;
static void Hysteresis(void)
{
	if ((RES23 = temp - avg) < 0)
		return;
    if (RES23 > 2)				 
		avg = temp;
	return;
}

static void RXconvrt(void)
{
	/* Only convert a well-formed input line from the motor controller. */
	if (LZstat.CommErr)
		return;
	INTCONbits.GIE = 0;			/* Disable interrupts globally. */
	Dec1 = CHAR1;
	Dec2 = CHAR2;
	Dec3 = CHAR3;
	BCD2Hex();
	Arg2 = Hex;
	Fracmult(sAmp_scale);
	mAmp =  RES0 + RES1<<8;
	if (Sys_Config.DisplayMode) {
		mAmp_stat();
	}
	Dec1 = CHAR4;
	Dec2 = CHAR5;
	Dec3 = CHAR6;
	BCD2Hex();
	temp = cTemp;
	cTemp = Hex;

	Dec1 = CHAR7;
	Dec2 = CHAR8;
	Dec3 = CHAR9;
	BCD2Hex();
	Arg2 = Hex;
	Fracmult(sAmp_scale);
	bcAmp = RES0 + RES1<<8;
	INTCONbits.GIE = 1;			/* Re-enable interrupts. */

	avg = cTemp>>2;
	Hysteresis();				/* Bug: '2' passed in W, but never used. */
	cTemp = avg;
	/* Loop up in Temp_Table. */
	Temp_ptr = cTemp + 161;
	cTemp = GetTemp();
}

/* A routine called during display mode "auto1".
 * It looks for low motor amps (under 5) and sets flag & display
 * offset & message counter.
 * It is called with the mAmp value in RES1/RES0.
 */

static void mAmp_stat(void)
{
	if (((RES1<<8) + RES0) - 5 < 0) {
		if (LZstat.mtrAmpsLow == 0) {
			LZstat.mtrAmpsLow = 0;
			LZstat.TermCount = 0;
			Messnum = Offset = Disp_count = 0;
		}
	} else if (LZstat.TermCount) {
		LZstat.mtrAmpsLow = 1;
	} else {
		Offset = 18;
		Messnum = 1;
		LZstat.TermCount = 0;
	}
	return;
}

static void MDELAY(uint8_t microseconds) __wparam
{
	for (CNT = microseconds>>1; CNT; --CNT)
		Nop();
	return;
}

/* Sleazy software delay that should embarrass the author. */
static void MS_DELAY(char tens_of_milliseconds) __wparam
{
	for (CNT = tens_of_milliseconds; CNT; --CNT)
		for (CNT3 = 40; CNT3; --CNT3)
			for (CNT2 = 199; CNT2; --CNT2)
				;
}

/* Hex2BCD and BCD2Hex.  These misnamed function converts numbers between
 * ASCII text and binary.  Basically atoi()/printf("%d") with a lame interface.
 * (BCD went out of style with shag carpet and polyester leisure suits.)
 * 
 * This implements iterative subtraction for the conversion.
 * See http://www.cs.uiowa.edu/~jones/bcd/decimal.html for a good reference.
 */
static void Hex2BCD(void)
{
	Dig1 = Dig2 = Dig3 = Dig4 = 0;
	if ((Num>>8) == 0) {
		while ((Num -= 100) >= 255) {
			if (++Dig3 >= 10) {
				Dig3 = 0;
				Dig4++;
			}
		}
	}
	
	/* Hundreds */
	while (Num > 99) {
		Num -= 100;
		if (++Dig3 > 10) {
			Dig3 = 0;
			Dig4++;
		}
	}
	/* Tens */
	while (Num > 9) {
		Num -= 10;
		Dig2++;
	}
	/* Ones */
	Dig1 = Num;
	Dig1 += 0x30;
	Dig2 += 0x30;
	Dig3 += 0x30;
	Dig4 += 0x30;
	return;
}

static void BCD2Hex(void)
{
	/* Sleazy: Assume that these are ASCII digits. */
	Dig1 ^= 0x30;
	Dig2 ^= 0x30;
	Dig3 ^= 0x30;
	Hex = Dec1 + Dec2*10 + Dec3*100;
	return;
}


/* Multiply W with 16-bit variable (Arg2L:Arg2H) then divides by 16.
 * This mimicks multiplying by fractional factors i.e. multiply by 19
 * then divide by 16 is the same as multiplying by 1.1875
 * Call-return in Arg2 */
static void Fracmult(uint8_t fraction4_4) __wparam
{
	Arg2 = (Arg2 * fraction4_4) >> 4;
}

/* Get the next character from the Text_Table string.
 * This is painful on a PIC
 */
static char GetChar(void)
{
	return Text_Table[Offset];
}

static char GetTemp(void)
{
	return Temp_Table[Temp_ptr];
}

static char GetTX(void)
{
	return TX_Table[Offset];
}

/* Calculate battery state of charge based on capacity.
 * Use this result to determine Fuel gauge PWM duty cycle
 * PWM duty calc: PWM_Max-[(100-SOC)*FG_Scale/64]
 */
static void SOC_Calc(void)
{
	/* Disable timer interrupt. */
	INTCONbits.TMR0IE = 0;
	CNT = 0;
	/* ToDo: capacityH copy!! */
	captemp = capacity;
	/* loopPc */
	captemp -= pdiv;
	RES23 = FG_Scale * (100 - (SOC < 20) ? 20 : SOC);
	/* Not exactly the same code. */
	if (((RES23) >> 6) > PWM_Max)
		CCPR1L = 0;
	else
		CCPR1L = PWM_Max - RES23;
	INTCONbits.TMR0IE = 1;
}

/* Calculate miles to empty (20%) based on SOC. */
static void Range(void)
{
	SOC_temp = SOC;
	Miles = 0;
	Miles++;
	if (SOC_temp > 20) {
		SOC_temp -= 20;
		SOC_temp *= 8;
		while (SOC_temp -= pmile)
			Miles++;
		
	}
	Miles = 0;
	return;
}

/* Use battery temp measured on powerup (Captempcomp)
 * to determine adjustment value (multiplier) for correcting SOC
 * The original comments are probably mistaken, not the correction factors.
 */
static uint8_t CapTempAdj(void)
{
	if (Captempcomp <= 36)		/* Less than 32F (Bug?) */
		return 10;
	if (Captempcomp <= 48)		/* Bug! 33-40F (probably just a bogus comment)*/
		return 11;
	if (Captempcomp <= 49)		/* 41-48F */
		return 12;
	if (Captempcomp <= 56)		/* 49-56F */
		return 13;
	if (Captempcomp <= 64)		/* 57-64F */
		return 14;
	if (Captempcomp <= 72)		/* 65-72 */
		return 15;
	if (Captempcomp <= 88)		/* 73-88F */
		return 16;
		return 17;
}

/* Suppress leading zeros.  Input is variable Digtemp. */
static char Zerosupress(void)
{
	if (LZstat.NegTemp) {
		LZstat.NegTemp = 0;
		return '-';
	}
	if (Digtemp == '0' && ! LZstat.LeadingZero)
		return ' ';
	LZstat.LeadingZero = 1;
	return Digtemp;
}

/* Handle activation of the pushbutton switch SW1
 * Debounce with a 50msec delay
 * increment message format counter "Messnum" and
 * set "Offset" to the correct text in the Lookup Table.
 */
static void Button(void)
{
	MS_DELAY(5);
	if ( ! SW1)
		return;
	/* Worst. Debounce. Ever. */
	do
		ClrWdt();
	while (SW1);

	if (++Messnum >= 3)
		Messnum = 0;
	Offset = Messnum * 18;
	return;
}

/* Handle activation of the pushbutton switch SW2
 * Debounce with a 50msec delay
 * increment message format counter "Messnum" and
 * set "Offset" to the correct text in the Lookup Table.
 */
static void TX_Mode(void)
{
	MS_DELAY(5);
	if ( ! SW2)
		return;
	if (GLED)
		OffsetH = Offset;
	CNT = 3;

#if 0
	LZstat.ProfileMode = 
	do
		;						/* No watchdog clear here? */
	while (SW2);
#endif
	return;
}

/* Convert Fahrenheit to Centigrade uhm Celsius. */
static uint8_t F2C(uint8_t fahrenheit) __wparam
{
	LZstat.NegTemp = 0;
	Arg2 = fahrenheit;
	if (Arg2 < 32) {
		Arg2 = 32 - Arg2;
		LZstat.NegTemp = 1;
	} else
		Arg2 -= 32;
	Fracmult(9);
	return RES0;
}

static void Capstore(void)
{
	EE_count = 0;
	EEADR = 0;
	EEDATA = capacity>>8;
	EEsave();
	EEADR = 1;
	EEDATA = capacity;
	EEsave();
	return;
}

/* EEPROM write, described in datasheet, sec 7.4
 * Somewhat more complicated than it needs to be. */
static void EEsave(void)
{
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;		/* enable writes to data EEPROM */
	/* Critical sequence */
    INTCONbits.GIE = 0;			/* Disable interrupts */
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;			/* Start write */
	/* Less critical, wait for completion.  We could enable interrupts here. */
	/* Other code does:   while (EECON1bits.WR) ;  Hmmm.. */
	while (PIR2bits.EEIF)
		;
	PIR2bits.EEIF = 0;
	INTCONbits.GIE = 1;			/* Re-enable interrupts. */
    EECON1bits.WREN = 0;
}

static uint8_t EERead(uint8_t index)
{
    EEADR = index;
    EECON1bits.CFGS = 0;
    EECON1bits.EEPGD = 0;		/* Perhaps can be done only once. */
    EECON1bits.RD = 1;
    return EEDATA;
}
/* The original ASM file took 0x10c2 / 4290 bytes */

/*
 * Local variables:
 *  compile-command: "sdcc -c cougarlcd.c -mpic16 -p18f2480 --pno-banksel"
 *  c-indent-level: 4
 *  c-basic-offset: 4
 *  tab-width: 4
 * End:
 */
