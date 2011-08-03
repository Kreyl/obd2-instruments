

/* Embed the configuration word within the C source file.
 * See the data sheet for the meaning of the configuration word bits.
 */
/* CONFIG1H */
#if 0
  /* 
CONFIG OSC = HS, FCMEN = OFF, IESO = OFF
CONFIG2L
     CONFIG PWRT = ON, BOREN = OFF, BORV = 2
     ;Setup CONFIG2H
     CONFIG WDT = OFF, WDTPS = 1024
     ;Setup CONFIG3H
     CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF
     ;Setup CONFIG4L
     CONFIG STVREN = OFF, LVP = OFF, XINST = OFF, DEBUG = OFF
     ;Setup CONFIG5L
     CONFIG CP0 = OFF, CP1 = OFF
     ;Setup CONFIG5H
     CONFIG CPB = OFF, CPD = OFF
     ;Setup CONFIG6L
     CONFIG EBTR0 = OFF
     ;Setup CONFIG6H
     CONFIG EBTR1 = OFF
     ;Setup CONFIG7L
  */
#endif

/* I/O EQUATES AND DEFINES */
#define SW2    PORTA,4
#define SW1    PORTB,0
#define RLED   PORTC,0
#define GLED   PORTC,1
#define ALRM	PORTC,3
#define E	PORTC,5		/* Enable line on LCD */
#define	RS	PORTC,4		/* Instruction/data select */
#define LCD  	PORTB		/* LCD data port */
enum LCD_commands {
  LCD_ClrScr=1, LCD_Home=2, LCD_On=12, LCD_off=8,
  LCD_Left=24, LCD_Right=28, LCD_Line2=0xC0,
};

/* Constants used for MIN/MAX or calculations */
const MT_Max = 200;		/* Max motor temp for error LED */
const CT_Max = 167;		/* Max controller temp for error LED */
const BT_Max = 125;		/* Max battery temp for error LED */
 /* Traction battery capacity in Amp/seconds (mid byte of 24-bit) */
const bcapacityM = 0x5E;


void setup_io_ports(void)
{
  /* Select primary oscillator */
  OSCCON = 0x74;
  PORTA = PORTB = PORTC = 0;


char text_table[] = "Volts mAmp bAmp \0"
"aVolt  SOC  MTE \0"
"Tmot Tcont Tbat \0"
" RESETTING SOC \0"
"cTime Amps SOC \0";

char TX_Table[] = "rtd-period \0"
"200\0"
"c-rr 15\r"
"bat-amps-lim 350\r\0"
"c-rr 5\r"
"bat-amps-lim 190\r";


/* Omitted: peukert multiplier tables. */

/* Initialize the LCD */
void init_lcd(void)
{
command8(0x30);			/* Software reset */
ms_delay(10);			/* Delay 100ms */
command8(0x30);			/* Reset again */
ms_delay(10);			/* Delay 100ms */
command8(0x30);			/* Reset again */
ms_delay(10);			/* Delay 100ms */

command8(0x28);			/* Set display to 4 bit mode */
command(0x28);			/* Now, in 4 bit mode, set to two lines. */
command(ON);
command(6);			/* Shift along every write */
}

/* Set the LCD to the second line. */
static inline void second_line(void)
{
  command(LIN2);
}
