/*************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
File:     $Id: uart.c,v 1.6.2.1 2007/07/01 11:14:38 peter Exp $
Software: AVR-GCC 4.1, AVR Libc 1.4.6 or higher
Hardware: any AVR with built-in UART, 
License:  GNU General Public License 
          
DESCRIPTION:
    An interrupt is generated when the UART has finished transmitting or
    receiving a byte. The interrupt handling routines use circular buffers
    for buffering received and transmitted data.
    
    The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE variables define
    the buffer size in bytes. Note that these variables must be a 
    power of 2.
    
USAGE:
    Refere to the header file uart.h for a description of the routines. 
    See also example test_uart.c.

NOTES:
    Based on Atmel Application Note AVR306
                    
LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
                        
*************************************************************************/

/************************************************************************
uart_available, uart_flush, uart1_available, and uart1_flush functions
were adapted from the Arduino HardwareSerial.h library by Tim Sharpe on 
11 Jan 2009.  The license info for HardwareSerial.h is as follows:

  HardwareSerial.cpp - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modified 23 November 2006 by David A. Mellis
************************************************************************/

/************************************************************************
Changelog for modifications made by Tim Sharpe, starting with the current
  library version on his Web site as of 05/01/2009. 

Date        Description
=========================================================================
05/11/2009  Changed all existing UARTx_RECEIVE_INTERRUPT and UARTx_TRANSMIT_INTERRUPT
              macros to use the "_vect" format introduced in AVR-Libc
			  v1.4.0.  Had to split the 3290 and 6490 out of their existing
			  macro due to an inconsistency in the UART0_RECEIVE_INTERRUPT 
			  vector name (seems like a typo: USART_RX_vect for the 3290/6490
			  vice USART0_RX_vect for the others in the macro).
			Verified all existing macro register names against the device
			  header files in AVR-Libc v1.6.6 to catch any inconsistencies.
05/12/2009  Added support for 48P, 88P, 168P, and 328P by adding them to the
               existing 48/88/168 macro.
			Added Arduino-style available() and flush() functions for both
			supported UARTs.  Really wanted to keep them out of the library, so
			that it would be as close as possible to Peter Fleury's original
			library, but has scoping issues accessing internal variables from
			another program.  Go C!
05/13/2009  Changed Interrupt Service Routine label from the old "SIGNAL" to
               the "ISR" format introduced in AVR-Libc v1.4.0.

************************************************************************/

#include "uart.h"

#ifdef USE_UART0

#if !defined(uart0)
#error "No uart0 definition found for this microcontroller!!"
#endif

ISR(UART0_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
	uart0::receive_handler();
}


ISR(UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
	uart0::transmit_handler();
}

#endif  //uart0

#ifdef USE_UART1

#if !defined( uart1 )
#error "No uart1 definition found for this microcontroller!"
#endif

ISR(UART1_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART1 Receive Complete interrupt
Purpose:  called when the UART1 has received a character
**************************************************************************/
{
	uart1::receive_handler();
}


ISR(UART1_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART1 Data Register Empty interrupt
Purpose:  called when the UART1 is ready to transmit the next byte
**************************************************************************/
{
	uart1::transmit_handler();
}

#ifdef USE_UART2

#if !defined( uart2 )
#error "No uart2 definition found for this microcontroller!"
#endif

ISR(UART2_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART1 Receive Complete interrupt
Purpose:  called when the UART1 has received a character
**************************************************************************/
{
	uart2::receive_handler();
}


ISR(UART2_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART1 Data Register Empty interrupt
Purpose:  called when the UART1 is ready to transmit the next byte
**************************************************************************/
{
	uart2::transmit_handler();
}

#endif

#ifdef USE_UART3

#if !defined( uart3 )
#error "No uart3 definition found for this microcontroller!"
#endif

ISR(UART3_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART1 Receive Complete interrupt
Purpose:  called when the UART1 has received a character
**************************************************************************/
{
	uart3::receive_handler();
}


ISR(UART2_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART1 Data Register Empty interrupt
Purpose:  called when the UART1 is ready to transmit the next byte
**************************************************************************/
{
	uart3::transmit_handler();
}

#endif

#endif