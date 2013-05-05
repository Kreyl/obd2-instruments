/* vvvv-stm32.c: STM32 specific routines for the VVVVroom motor controller. */
/*
  Written 2010-2011 by Donald Becker and William Carlson.

  These are the hardware interface routines for STM32 version of the
  VVVVroom motor controller.  While the entire design and structure is
  influenced by the features and performance of the specific controller
  chip, these are the functions that are most intimately tied to the
  specific device's registers and bits.

	This file contains the following subsystems
	Configuration data read and write, using flash instead of EEPROM
	Processor initialization, including clock configuration
	I/O port configuration
	A/D and PWM-timer configuration
	SPI initialization.

	Some hardware-specific details remain in the other subsystem code
	for convenience
	Updating the PWM percentage
	Reading the A/D converter values
	SPI reads and write
*/
static const char versionA[] =
"$Id: vvvv-stm.c $\n";

#if defined(STM32)
#include "armduino.h"
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#include "vvvvroom.h"
#include "can.h"

/* Flag we are using the 32f100 thus the STM32VL-Discovery board. */
int8_t is_vl_discovery = 0;

/* Check that the program in flash matches the original.
 * This requires that the CRC immediately follow the program in flash,
 * resulting in an overall sum of -1.
 * For the Arduino we use the 0x8408 polynomial, the same as used in PPP.
 * For STM processors we the built-in 32 bit polynomial, as used in Ethernet.
 */
const uint32_t __attribute__((weak)) firmware_crc = 0;

int check_firmware_integrity(void)
{
	uint32_t *pgm_addr;

	AHBENR |= 0x40;			/* Turn on the CRC clock. */
	CRC_CR = 1;					/* Reset the CRC unit. */
	for (pgm_addr = (void *)0x08000000; pgm_addr < (uint32_t*)firmware_crc;
		 pgm_addr++)
		CRC_DR = *pgm_addr;
	return ! (firmware_crc && firmware_crc != CRC_DR);
}

/* The Arduino used EEPROM for configuration settings.  With EEPROM,
 * words can be individually updated, and it's reasonable to write multiple
 * copies.
 * With the STM32 we have only the program flash for persistent storage.
 * Flash needs to be erased a page at a time, but can be read as easily
 * as other program memory.
 */


/* Read the configuration structure from the "EEPROM".
 * If the magic number and CRC matches, use it directly.
 * Otherwise use the default config, without warning.
 */
void read_config(void)
{
	config = default_config;
	return;
}

/*
 * Writing the config is done to the next unwritten slot in the
 * flash.  If all slots have been written, erase whole block and
 * start from the first slot.
 * We calculate the config CRC just before writing.
 */
void write_config(void)
{
#if 0
	config.crc = calc_block_crc(sizeof(config) - sizeof(unsigned),
								(void *)&config);
	/* This might take a long time, so take the risk. */
	watchdog_disable();
	eeprom_write_block(&config, (void *)EE_CONFIG_ADDRESS, sizeof(config));
	watchdog_enable();
#endif
}

/* Watchdog section.  Ready for future functionality. */
void wdt_reset(void)
{
	WWDG_CR = 0x7F;
}
void watchdog_disable(void)
{
	/* The watchdog cannot be disabled on the STM32. */
}
void watchdog_enable(void)
{
	WWDG_CR = WDGA | 0x7F;
}

/* Time functions.
 * The time is based on 1KHz interrupts of the ARM SysTick counter.
 * With an unsigned 32 bit value we can span about 50 days before
 * rolling over.
 *
 * The power-up value in SYSTICK_STCALIB is documented by ARM to produce
 * a 10msec tick with the default clock.  STM apparently didn't
 * understand the purpose and used a value of 9000.  The only place that
 * value makes sense is producing a 1 msec tick with the subset of chips
 * that have a 72MHz max internal PLL.  And if you can configure STM's
 * elaborate clock tree for that, you didn't need the default.
 *
 * We configure Systick to use the "external reference clock", which is
 * /8 of the core clock.  When the core clock is 24MHz, this produces a
 * 3MHz signal.  
 */
volatile uint32_t clock_1msec;
void setup_clock(void)
{
	SYSTICK_CNT = clock_1msec = 0;
	SYSTICK_ARR = 3000;		/* Should be SYSTICK_ARR = SYSTICK_STCALIB; */
	SYSTICK_CR = 0x03;			/* Enable counter and interrupt. */
}

/* Does NOT get an ISR() wrapper. */
void SysTick_Handler(void)
{
	clock_1msec++;
}

/* Configure the I/O port functions and directions.
 * This changes with the hardware platform and specific MCU part.
 * We need to set up the PWM timer, clock timer, ADC, LED pins,
 * SPI, and perhaps USART.
 * USART2, PA02=USART2_Tx PA03=USART2_Rx.
 * USART3 remapped, PC10=USART3_Tx PC11=USART3_Rx.
 *
 * Port pin settings: function, direction, pull-up and initial values.
 *  0 Analog input
 *  1 General purpose output, medium speed.
 *  4 Floating input (default)
 *  B Alternate function output, fastest (9=10MHz, A=2MHz, B=50MHz)
 */

/* Disable the I2C outputs to avoid the SPI1 remap errata.
 * It's not clear that this works. */
unsigned int _RCC_APB1ENR =
	0x7fffffff & ~APB1ENR_I2C1EN & ~APB1ENR_I2C2EN;

void setup_io_ports(void)
{

	/* Use the non-zero bits in APB1ENR to deduce the chip and thus board.
	 *  The 32F100 returns 0x78?64837  (we disable I2C1)
	 *  The 32F105 returns 0x3e5?c83f.
	 * Set the clock tree based on the result.
	 * Note: If running >24MHz we may need to increase the flash wait states.
	 * 0 for 0-24 MHz, 1 for 24-48 MHz, and 2 for 48-72 MHz. */
	if ((APB1ENR & 0xffff) == 0x4837) {
		is_vl_discovery = 1;
		/* For the 32F100 Set the clock multiplier to 3x, 24MHz.
		 * ADC:12MHz APB:24MHz, all clock dividers /1. */
		RCC_CR = 0x00010000;		/* Enable External clock */
		RCC_CFGR = 0x00050002;		/* PLL to 3x */
		RCC_CR = 0x01090000;		/* Switch to PLL */
	} else {
		/* For the 32F103  Set the PLL multiplier to 6x, 24MHz.
		 * PCLK1 is 24MHz
		 * PCLK2 is 24MHz
		 * AHB   is 24MHz
		 * ADC   is 12MHz
		 */
		RCC_CFGR = 0x00000000;		/* Use internal clock to start. */
		RCC_CR  &= ~0x01FF0000;		/* Enable External clock */
		RCC_CR  |= 0x00090000;		/* Enable External clock */
		RCC_CFGR = 0x00130002;		/* PLL to 6x  HSE/2 */
		RCC_CR  |= 0x01090000;		/* Switch to PLL */
	}

	/* This works around a chip bug with SPI1 remapped: the I2C1
	 * function must be completely disabled or it blocks the MOSI
	 * output.  This is not a simple output conflict, as neither
	 * remapping the I2C port nor leaving it disabled avoids the
	 * problem.
	 */
	APB1ENR &= ~APB1ENR_I2C1EN;

	/* Pin remap
	 * 0x0200---- JTAG (SWJ) set to SWD-only to free PB3/4 for SPI1 or PWM
	 * TIM3 remap
	 *  0x000 no remap CH1/2/3/4 are PA6/PA7/PB0/PB1
	 *  0x800 remap 2  TIM3 PB4/PB5/PB0/PB1 for QAR PWM mosfet outputs
	 *  0xc00 remap 3  TIM3 PC6/PC7/PC8/PC9 for Discovery LEDs on PC8/PC9
	 * 0x00 USART3 standard Tx=PB10 Rx=PB11
	 * 0x10 USART3 to partial remap Tx=PC10 Rx=PC11 Ck=PC12.
	 * 0x01 SPI1 remap, using PB3/4/5 CLK/MISO/MOSI w/ PA15 for CS
	 */
	if (is_vl_discovery)
		AFIO_MAPR |= 0x02000C11;
	else
		AFIO_MAPR |= 0x02000810;

	/* Configure PortA pins.  Most are A/D conversion, ADC0..7 on PA0..7. */
	/* UART2, temp debugging I/O for QAR, is on PA2/PA3 */
	/* SPI1 is on PA5/PA6/PA7 (not used) */
	GPIOA_CRL = 0x00004B00;

	/* PA8/TIM1_CH is the primary motor PWM output.
	 * PA9/10 are USART1 Tx/Rx
	 * PA11/PA12 are CANRx/Tx
	 * PA15 is SPI1_SS
	 */
	GPIOA_CRH =  0x144A444B;
	GPIOA_BSRR = 0x80000000;	/* Set PA15 high immediately. */

	/* PB0/1 are ADC8/9, PB6/7 are USART1-remap Tx/Rx or CAN.
	 * PB0 ADC8 disabled in favor of the QAR LED output GPIO/TIM3_CH3.
	 * PB2 is boot1, a jumper on the Discovery board, N/C on QAR
	 * PB3 is PWM3 (#4) on the QAR board, TIM2-CH2-remap
	 * PB4 is PWM4 (#3) on the QAR board, TIM3-CH1-remap
	 * PB5 is PWM5 (#2), TIM3-CH2-remap (unusable because of a chip bug)
	 * PB6 is PWM6 (top) on the QAR board, TIM4-CH1
	 */
	if (is_vl_discovery) {
		/* The SPI configuration is problematic, due to a conflict with
		 * other functions e.g. TIM1_CH1N or ADC4-7.  We are currently using
		 * SPI1-remap and turning off the JTAG interface pins.
		 * SPI1:	   SPI1_SCK:PA5  SPI1_MISO:PA6  SPI1_MOSI:PA7  SS:PA4
		 * SPI1-remap: SPI1_SCK:PB3  SPI1_MISO:PB4  SPI1_MOSI:PB5  SS:PA15 
		 * SPI2:       SPI2_SCK:PB13 SPI2_MISO:PB14 SPI2_MOSI:PB15 SS:PB12 
		 */
		GPIOB_CRL = 0x00B8B000;
		GPIOB_BSRR = 0x00000010;	/* Pull-up on MISO */
		/* Turn on: SPI enable, 8 bit, MSB first, 0,0, Master, F_OSC/4. */
		SPI_CR1 = 0x4300 | SPI_SPE | SPI_CLK_DIV | SPI_MSTR;
		SPI_CR2 = 0;
	} else
		GPIOB_CRL = 0x0A1AA001;				/* Was 0x0A1AA00A */
	/* Set PB13 as an output for TIM1_CH1N. */
	GPIOB_CRH = 0x00B00000;

	/* ADC10..15 on PC0..5.  Note: Not on 36/48 pin packages.
	 * PC6/PC7 are TIM3_CH1/2 outputs, currently unused. */
	GPIOC_CRL =  0x44000000;

	/* Discovery blue LED is pin PC8/TIM3_CH3, green LED is pin PC9/TIM3_CH4
	 * TIM3-remap3 CH-1/2/3/4 are on PC6/7/8/9.
	 * PC8 to alternate function (Timer 3 Ch 3), PC9 to GPIO output.
	 * PC10 is USART3_Tx alternate function output
	 * PC11 is USART3_Rx input w/pull-up.
	 * PC12 is unused, PC13 is RTC-tamper, PC14/PC15 default to OSC32
	 */
	GPIOC_CRH =  0x44448AB1;
	GPIOC_BSRR = 0x0B00;		/* Pull-up on PC11.  PC8/9 LEDs on. */

	return;
}

/*
 * Setup timers for motor PWM and tachometer input.
 */
void setup_timers(void)
{

	/* Configure timer 1 as the motor control PWM signal. */
	TIM1_CR2 = 0x0000;			/* Break output level is 0. */
	TIM1_CCMR1 = 0x0060;	/* Ch1 mode 1, buffered CCR1 */
	TIM1_CCMR2 = 0x0000;	/* Ch3/4 off for now*/
	TIM1_RCR = 0;			/* Update compare count every cycle.  */
	/* Zero prescale, count up/down by 1000 (12KHz from 24MHz PLL). */
	TIM1_PSC = 0;
	TIM1_ARR = 1000;
	TIM1_CCR1 = TIM1_CCR2 = 0;			 /* Start out at zero. */
	TIM1_CCER = 0x0005;			/* Enable CH1+1N, active high. */
	TIM1_BDTR = 0x8000;			/* Break and deadtime settings. */
	TIM1_DIER = 0x0001;			/* Enable interrupt on update. */
	TIM1_EGR = 1;				/* Trigger a load of buffered settings.  */
	/* Before we start the timer we have to provide an A/D result to read. */
	start_adc_conversion(ADC_CHANNEL_M_CURRENT);
	TIM1_CR1 = 0x65;
	INTR_SETENA(TIM1_UP_TIM16_Intr);

	/* We configure Timer 3 for PWM output
	 * On the Discovery, TIM3 CH-1/2/3/4 are on PC6/7/8/9
	 *  The blue LED is pin PC8 / TIM3_CH3, green LED is pin PC9 / TIM3_CH4
	 * On QAR, TIM3 is PB4/PB5/PB0/PB1 for PWM mosfet outputs
	 *  CH1 is PB4, PWM4-mosfet (#3)
	 *  CH2 is PB5, PWM5-mosfet (#2)
	 *  CH3 is PB0, fault LED
	 *  CH4 is N/C
	 */
	TIM3_CR1 = 0x05;
	TIM3_CCMR1 = 0x6060;			/* Configure channel 1+2 */
	TIM3_CCMR2 = 0x6060;			/* Configure channel 3+4 */
	/* No prescaler, count up 960 (25KHz from the 24MHz PLL).
	 * This is the PWM rate for PC fans and is reasonable for other uses. */
	TIM3_ARR = 960;
    /* Do we need to force a reload with TIM3_EGR = TIM_EGR_UG ? */
	TIM3_CCR1 = TIM3_CCR2 = 0;
	TIM3_CCR3 = TIM3_CCR4 = 0;
	TIM3_CCER = 0x1311;			/* Enable channel 4/3(inverted)/2/1 */

	/* Timer 4 as a general-purpose PWM timer
	 * CH1 is PB6, the QAR top open-drain MOSFTET output.
	 */
	TIM4_CR1 = 0x05;
	TIM4_CCMR1 = 0x6060;			/* Configure channel 1+2 */
	TIM4_CCMR2 = 0x6060;			/* Configure channel 3+4 */
	/* No prescaler, count up 24000 (1KHz from the 24MHz PLL).
	 * A low frequency output may have better efficiency,
	 * but risks audible noise. */
	TIM4_ARR = 24000;
	TIM4_CCR1 = TIM4_CCR2 = 0;
	TIM4_CCR3 = TIM4_CCR4 = 0;
	TIM4_EGR = 1;
	TIM4_CCER = 0x0011;			/* Enable channel 2/1 */

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
