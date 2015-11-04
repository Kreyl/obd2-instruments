This project contains multiple subsystems for implementing a custom
automotive network using CAN bus, including OBD2 and status reporting,
and converting CAN bus reported parameter into gauge or screen outputs.

The code supports multiple popular microcontrollers in the Atmel AVR and
STMicro STM32 family, and common CAN bus controllers.

The STM32 directory contains code for the STM32F100 series
microcontrollers.  It has device support for both the MCP2515 SPI CAN
controller and the bxCAN controller built into the f103, f105 and f107
processors.

The AVR directory contains code for the ATMEGA AVR series of
microcontrollers.  It had tested device support for the MCP2515 SPI CAN
controller, and preliminary (nominally complete but unverified)
support for the built-in CAN controller on the AT90CAN32/64/128
microcontrollers.

The Cougar directory contains add-on code for the Cougar EV motor
controller firmware.  It is designed to be added onto the existing
firmware source code.  It branched from a November 2010 version of the
system and has fewer features in order to minimize the program space
taken on the Cougar controller's ATMEGA8-16U processor.  Notably it
reports only current values and does not log faults.

This code started as the firmware of our 'QAR' EV motor controller.
Over time we expanded it into a more general reporting, gauge driver and
display toolkit.

Enjoy

Donald Becker
William Carlson
Q.M.C. -- "The best quantum mechanics in the fleet"