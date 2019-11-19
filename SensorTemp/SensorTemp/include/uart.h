/**
* @file uart.h
* Template class for handling multiples built-in uarts.
* @author Joao Leonardo Fragoso
* @date 2018-07-02
*/
#ifndef _UART_H_
#define _UART_H_
/** 
 *  @defgroup _comm Communication Library
 * 
 *  @brief Interrupt UART library using the built-in UART with transmit and receive circular buffers. 
 *
 *  This library can be used to transmit and receive data through the built in UART. 
 *
 *  An interrupt is generated when the UART has finished transmitting or
 *  receiving a byte. The interrupt handling routines use circular buffers
 *  for buffering received and transmitted data.
 *
 *  The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE constants define
 *  the size of the circular buffers in bytes. Note that these constants must be a power of 2.
 *  You may need to adapt this constants to your target and your application by adding 
 *  CDEFS += -DUART_RX_BUFFER_SIZE=nn -DUART_RX_BUFFER_SIZE=nn to your Makefile.
 *
 *  @note Based on Atmel Application Note AVR306
 *  @author Joao Fragoso 
 */
 
/**@{*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#if (__GNUC__ * 100 + __GNUC_MINOR__) < 304
#error "This library requires AVR-GCC 3.4 or later, update to newer AVR-GCC compiler !"
#endif

//#define __STR(x) #x
//#define _STR(x) __STR(x)

/*
 *  constants and macros
 */
#if defined(__AVR_AT90S2313__) \
|| defined(__AVR_AT90S4414__) || defined(__AVR_AT90S4434__) \
|| defined(__AVR_AT90S8515__) || defined(__AVR_AT90S8535__) \
|| defined(__AVR_ATmega103__)
/** 
* Old AVR classic or ATmega103 with one UART
*/
#define AT90_UART
/**
* Receive interruption vector definition.
*/
#define UART0_RECEIVE_INTERRUPT   UART_RX_vect
/**
* Transmit interruption vector definition.
*/
#define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
/**
* macro template definition for user.
*/
#define uart0 uart< ((uint16_t)&(USR)), ((uint16_t)&(UCR)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), 0, 0, ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRR)), ((uint16_t)&(UBRR), 0 >
#elif defined(__AVR_AT90S2333__) || defined(__AVR_AT90S4433__)
/* old AVR classic with one UART */
#define AT90_UART
#define UART0_RECEIVE_INTERRUPT   UART_RX_vect
#define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSRA)), ((uint16_t)&(UCSRB)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), 0, 0, ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRR)), ((uint16_t)&(UBRR)), 0 >
#elif  defined(__AVR_ATmega8__)  || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) \
|| defined(__AVR_ATmega323__)
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART_RXC_vect
#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSRA)), ((uint16_t)&(UCSRB)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), ((uint16_t)&(UCSRC)), _BV(URSEL)|_BV(UCSZ1)|_BV(UCSZ0), ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRRH)), ((uint16_t)&(UBRRL)), U2X >
#elif  defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__)
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSRA)), ((uint16_t)&(UCSRB)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), ((uint16_t)&(UCSRC)), _BV(URSEL)|_BV(UCSZ1)|_BV(UCSZ0), ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRRH)), ((uint16_t)&(UBRRL)), U2X >
#elif defined(__AVR_ATmega163__)
/* ATmega163 with one UART */
#define ATMEGA_UART
#define UART0_RECEIVE_INTERRUPT   UART_RX_vect
#define UART0_TRANSMIT_INTERRUPT  UART_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSRA)), ((uint16_t)&(UCSRB)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), 0, 0, ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRRHI)), ((uint16_t)&(UBRR)), U2X >
#elif defined(__AVR_ATmega162__)
/* ATmega with two USART */
#define ATMEGA_USART
#define ATMEGA_USART1
#define UART0_RECEIVE_INTERRUPT   USART0_RXC_vect
#define UART1_RECEIVE_INTERRUPT   USART1_RXC_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(URSEL0)|_BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#define uart1 uart< ((uint16_t)&(UCSR1A)), ((uint16_t)&(UCSR1B)), _BV(RXCIE1)|_BV(RXEN1)|_BV(TXEN1), ((uint16_t)&(UCSR1C)), _BV(URSEL1)|_BV(UCSZ11)|_BV(UCSZ10), ((uint16_t)&(UDR1)), UDRIE1, (_BV(FE1)|_BV(DOR1)), ((uint16_t)&(UBRR1H)), ((uint16_t)&(UBRR1L)), U2X1 >
#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
/* ATmega with two USART */
#define ATMEGA_USART
#define ATMEGA_USART1
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#define uart1 uart< ((uint16_t)&(UCSR1A)), ((uint16_t)&(UCSR1B)), _BV(RXCIE1)|_BV(RXEN1)|_BV(TXEN1), ((uint16_t)&(UCSR1C)), _BV(UCSZ11)|_BV(UCSZ10), ((uint16_t)&(UDR1)), UDRIE1, (_BV(FE1)|_BV(DOR1)), ((uint16_t)&(UBRR1L)), ((uint16_t)&(UBRR1L)), U2X1 >
#elif defined(__AVR_ATmega161__)
/* ATmega with UART */
#error "AVR ATmega161 currently not supported by this libaray !"
#elif defined(__AVR_ATmega169__)
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSRA)), ((uint16_t)&(UCSRB)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), ((uint16_t)&(UCSRC)), _BV(UCSZ1)|_BV(UCSZ0), ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRRH)), ((uint16_t)&(UBRRL)), U2X >
#elif defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__) || \
defined(__AVR_ATmega48P__) ||defined(__AVR_ATmega88P__) || defined(__AVR_ATmega168P__) || \
defined(__AVR_ATmega328P__)
/* TLS-Added 48P/88P/168P/328P */
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#elif defined(__AVR_ATtiny2313__)
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSRA)), ((uint16_t)&(UCSRB)), _BV(RXCIE)|_BV(RXEN)|_BV(TXEN), ((uint16_t)&(UCSRC)), _BV(UCSZ1)|_BV(UCSZ0), ((uint16_t)&(UDR)), UDRIE, (_BV(FE)|_BV(DOR)), ((uint16_t)&(UBRRH)), ((uint16_t)&(UBRRL)), U2X >
#elif defined(__AVR_ATmega329__) ||\
defined(__AVR_ATmega649__) ||\
defined(__AVR_ATmega325__) ||defined(__AVR_ATmega3250__) ||\
defined(__AVR_ATmega645__) ||defined(__AVR_ATmega6450__)
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#elif defined(__AVR_ATmega3290__) ||\
defined(__AVR_ATmega6490__)
/* TLS-Separated these two from the previous group because of inconsistency in the USART_RX */
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__)
/* ATmega with two USART */
#define ATMEGA_USART
#define ATMEGA_USART1
#define ATMEGA_USART2
#define ATMEGA_USART3
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
#define UART2_RECEIVE_INTERRUPT   USART2_RX_vect
#define UART3_RECEIVE_INTERRUPT   USART3_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
#define UART2_TRANSMIT_INTERRUPT  USART2_UDRE_vect
#define UART3_TRANSMIT_INTERRUPT  USART3_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#define uart1 uart< ((uint16_t)&(UCSR1A)), ((uint16_t)&(UCSR1B)), _BV(RXCIE1)|_BV(RXEN1)|_BV(TXEN1), ((uint16_t)&(UCSR1C)), _BV(UCSZ11)|_BV(UCSZ10), ((uint16_t)&(UDR1)), UDRIE1, (_BV(FE1)|_BV(DOR1)), ((uint16_t)&(UBRR1H)), ((uint16_t)&(UBRR1L)), U2X1 >
#define uart2 uart< ((uint16_t)&(UCSR2A)), ((uint16_t)&(UCSR2B)), _BV(RXCIE2)|_BV(RXEN2)|_BV(TXEN2), ((uint16_t)&(UCSR2C)), _BV(UCSZ21)|_BV(UCSZ20), ((uint16_t)&(UDR2)), UDRIE2, (_BV(FE2)|_BV(DOR2)), ((uint16_t)&(UBRR2H)), ((uint16_t)&(UBRR2L)), U2X2 >
#define uart3 uart< ((uint16_t)&(UCSR3A)), ((uint16_t)&(UCSR3B)), _BV(RXCIE3)|_BV(RXEN3)|_BV(TXEN3), ((uint16_t)&(UCSR3C)), _BV(UCSZ31)|_BV(UCSZ30), ((uint16_t)&(UDR3)), UDRIE3, (_BV(FE3)|_BV(DOR3)), ((uint16_t)&(UBRR3H)), ((uint16_t)&(UBRR3L)), U2X3 >
//#define uart3 uart< ((uint16_t)(0x130)), ((uint16_t)(0x131)), _BV(RXCIE3)|_BV(RXEN3)|_BV(TXEN3), ((uint16_t)(0x132)), _BV(UCSZ31)|_BV(UCSZ30), ((uint16_t)(0x136)), UDRIE3, (_BV(FE3)|_BV(DOR3)), ((uint16_t)(0x135)), ((uint16_t)(0x134)), U2X3 >
#elif defined(__AVR_ATmega644__)
/* ATmega with one USART */
#define ATMEGA_USART
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__)
/* ATmega with two USART */
#define ATMEGA_USART
#define ATMEGA_USART1
#define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
#define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
#define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
#define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
#define uart0 uart< ((uint16_t)&(UCSR0A)), ((uint16_t)&(UCSR0B)), _BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0), ((uint16_t)&(UCSR0C)), _BV(UCSZ01)|_BV(UCSZ00), ((uint16_t)&(UDR0)), UDRIE0, (_BV(FE0)|_BV(DOR0)), ((uint16_t)&(UBRR0H)), ((uint16_t)&(UBRR0L)), U2X0 >
#define uart1 uart< ((uint16_t)&(UCSR1A)), ((uint16_t)&(UCSR1B)), _BV(RXCIE1)|_BV(RXEN1)|_BV(TXEN1), ((uint16_t)&(UCSR1C)), _BV(UCSZ11)|_BV(UCSZ10), ((uint16_t)&(UDR1)), UDRIE1, (_BV(FE1)|_BV(DOR1)), ((uint16_t)&(UBRR1H)), ((uint16_t)&(UBRR1L)), U2X1 >
#else
#error "no UART definition for MCU available"
#endif


/** UART Baudrate Expression.
 * Compute baurate value for register initialization.
 * @param  baudrate baudrate in bps, e.g. 1200, 2400, 9600
 * @depends F_CPU
 */
constexpr uint16_t uart_baud_select(uint32_t baudRate) {
	return ((uint16_t)(((F_CPU)/16UL/(baudRate))-1UL));
};

/** UART Baudrate Expression for double speed.
 * Compute baurate value for register initialization when
 * setting double speed for true.
 * @param  baudrate baudrate in bps, e.g. 1200UL, 2400UL, 9600UL
 * @depends F_CPU
 */
constexpr uint16_t uart_baud_select_2x(uint32_t baudRate) {
	return ((uint16_t)(((F_CPU)/8UL/(baudRate))-1UL));
};

/*
 * @brief    Macro to automatically put a string constant into program memory
 */
//#define uart_puts_P(__s)       uart_puts_p(PSTR(__s))

/**
* Template for create a built-in uart handler.
* This templates create an interface class (all static - no object need) for
* handling a built-in uart. Correct register address will do the job. 
* And only one declaration for each uart to handle. Previous in this
* file, some macros are provided for most commons microcontrollers.
* @tparam STATUS_REG address of USART Control and Status Register A. example ((uint16_t)&(UCSR0A))
* @tparam CONTROL_REG address of USART Control and Status Register B.
* @tparam CONTROL_MASK bit mask for enabling 
*/
template < 
		   uint16_t STATUS_REG, 
           uint16_t CONTROL_REG,
		   uint8_t CONTROL_MASK,
           uint16_t CONTROL2_REG,
		   uint8_t CONTROL2_MASK,
		   uint16_t DATA_REG, 
		   uint8_t INT_PIN, 
		   uint8_t ERROR_MASK,
		   uint16_t BAUDRATE_REG_H,
		   uint16_t BAUDRATE_REG_L,
		   uint8_t DOUBLE_SPEED_PIN
		   >
class uart {

public:
/** Size of the circular receive buffer, must be power of 2 */
#ifndef UART_RX_BUFFER_SIZE
	static const uint8_t RX_BUFFER_SIZE=32;
	static const uint8_t RX_BUFFER_MASK=31;
#else
	#if ( (UART_RX_BUFFER_SIZE) & ((UART_RX_BUFFER_SIZE)-1) )
	#error RX buffer size is not a power of 2
	#endif
	static const uint8_t RX_BUFFER_SIZE=(UART_RX_BUFFER_SIZE);
	static const uint8_t RX_BUFFER_MASK=((UART_RX_BUFFER_SIZE)-1);
#endif
/** Size of the circular transmit buffer, must be power of 2 */
#ifndef UART_TX_BUFFER_SIZE
	static const uint8_t TX_BUFFER_SIZE = 32;
	static const uint8_t TX_BUFFER_MASK = 31;
#else
	#if ( (UART_TX_BUFFER_SIZE) & ((UART_TX_BUFFER_SIZE)-1) )
	#error TX buffer size is not a power of 2
	#endif
	static const uint8_t TX_BUFFER_SIZE = (UART_TX_BUFFER_SIZE);
	static const uint8_t TX_BUFFER_MASK = ((UART_TX_BUFFER_SIZE)-1);
#endif

/* test if the size of the circular buffers fits into SRAM */
//#if ( (UART_RX_BUFFER_SIZE+UART_TX_BUFFER_SIZE) >= (RAMEND-0x60 ) )
//#error "size of UART_RX_BUFFER_SIZE + UART_TX_BUFFER_SIZE larger than size of SRAM"
//#endif

/* 
** high byte error return code of uart_getc()
*/
	static const uint16_t FRAME_ERROR     = 0x0800;  /* Framing Error by UART       */
	static const uint16_t OVERRUN_ERROR   = 0x0400;  /* Overrun condition by UART   */
	static const uint16_t BUFFER_OVERFLOW = 0x0200;  /* receive ringbuffer overflow */
	static const uint16_t NO_DATA         = 0x0100;  /* no receive data available   */

/*
 *  module global variables
 */
	static volatile unsigned char TxBuf[TX_BUFFER_SIZE];
	static volatile unsigned char RxBuf[RX_BUFFER_SIZE];
	static volatile unsigned char TxHead;
	static volatile unsigned char TxTail;
	static volatile unsigned char RxHead;
	static volatile unsigned char RxTail;
	static volatile unsigned char LastRxError;
/*
** function prototypes
*/

/**
   @brief   Initialize UART and set baudrate 
   @param   baudrate Specify baudrate using macro UART_BAUD_SELECT()
   @return  none
*/
	static void init(uint16_t baudrate, bool enableDoubleSpeed=false)
	{
		TxHead = 0;
		TxTail = 0;
		RxHead = 0;
		RxTail = 0;
    
		#if defined( AT90_UART )
		/* set baud rate */
		_SFR_MEM8(BAUDRATE_REG_L) = ((unsigned char)(baudrate));
		/* enable UART receiver and transmitter and receive complete interrupt */
		//_SFR_MEM8(CONTROL_REG) = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);
		_SFR_MEM8(CONTROL_REG) = CONTROL_MASK;

		#elif defined (ATMEGA_USART)
		/* Set baud rate */
		if ( enableDoubleSpeed )
			_SFR_MEM8(STATUS_REG) = _BV(DOUBLE_SPEED_PIN);  //Enable 2x speed

		_SFR_MEM8(BAUDRATE_REG_H) = ((unsigned char)(baudrate>>8)); // & ~0x80;
		_SFR_MEM8(BAUDRATE_REG_L) = (unsigned char)(baudrate);
   
		/* Enable USART receiver and transmitter and receive complete interrupt */
		//_SFR_MEM8(CONTROL_REG) = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);
        _SFR_MEM8(CONTROL_REG) = CONTROL_MASK;
		/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
		_SFR_MEM8(CONTROL2_REG) = CONTROL2_MASK;

		#elif defined ( ATMEGA_UART )
		/* set baud rate */
		if ( enableDoubleSpeed )
			_SFR_MEM8(STATUS_REG) = _BV(DOUBLE_SPEED_PIN);  //Enable 2x speed
			//_SFR_MEM8(STATUS_REG) = _BV(U2X);  //Enable 2x speed

		_SFR_MEM8(BAUDRATE_REG_H) = (unsigned char)(baudrate>>8);
		_SFR_MEM8(BAUDRATE_REG_L) = (unsigned char)(baudrate);

		/* Enable UART receiver and transmitter and receive complete interrupt */
		//_SFR_MEM8(CONTROL_REG) = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);
		_SFR_MEM8(CONTROL_REG) = CONTROL_MASK;
				
		#endif

		
	};
	uart() {};
	~uart(){};

/**
 *  @brief   Get received byte from ringbuffer
 *
 * Returns in the lower byte the received character and in the 
 * higher byte the last receive error.
 * NO_DATA is returned when no data is available.
 *
 *  @param   void
 *  @return  lower byte:  received byte from ringbuffer1
 *  @return  higher byte: last receive status
 *           - \b 0 successfully received data from UART
 *           - \b NO_DATA           
 *             <br>no receive data available
 *           - \b BUFFER_OVERFLOW   
 *             <br>Receive ringbuffer overflow.
 *             We are not reading the receive buffer fast enough, 
 *             one or more received character have been dropped 
 *           - \b OVERRUN_ERROR     
 *             <br>Overrun condition by UART.
 *             A character already present in the UART UDR register was 
 *             not read by the interrupt handler before the next character arrived,
 *             one or more received characters have been dropped.
 *           - \b FRAME_ERROR       
 *             <br>Framing Error by UART
 */
	static unsigned int getc(void) {
		unsigned char tmptail;
		unsigned char data;

		if ( RxHead == RxTail ) {
			return NO_DATA;   /* no data available */
		}
    
		/* calculate /store buffer index */
		tmptail = (RxTail + 1) & RX_BUFFER_MASK;
		RxTail = tmptail;
    
		/* get data from receive buffer */
		data = RxBuf[tmptail];
    
		return (LastRxError << 8) + data;	
	};


/**
 *  @brief   Put byte to ring buffer for transmitting via UART
 *  @param   data byte to be transmitted
 *  @return  none
 */
	static void putc(unsigned char data) {
		unsigned char tmphead;
		tmphead  = (TxHead + 1) & TX_BUFFER_MASK;
		/* wait for free space into buffer */
		while ( tmphead == TxTail );
		TxBuf[tmphead] = data;
		TxHead = tmphead;
		/* enable UDRE interrupt */
		_SFR_MEM8(CONTROL_REG) |= _BV(INT_PIN);
	};

/**
 *  @brief   Put string to ring buffer for transmitting via UART
 *
 *  The string is buffered by the uart library in a circular buffer
 *  and one character at a time is transmitted to the UART using interrupts.
 *  Blocks if it can not write the whole string into the circular buffer.
 * 
 *  @param   s string to be transmitted
 *  @return  none
 */
	static void puts(const char *s ) {
	    while (*s)
		    uart::putc(*s++);
	};

/**
 * @brief    Put string from program memory to ring buffer for transmitting via UART.
 *
 * The string is buffered by the uart library in a circular buffer
 * and one character at a time is transmitted to the UART using interrupts.
 * Blocks if it can not write the whole string into the circular buffer.
 *
 * @param    s program memory string to be transmitted
 * @return   none
 * @see      uart_puts_P
 */
	static void puts_p(const char *progmem_s) {
		register char c;
		while ( (c = pgm_read_byte(progmem_s++)) )
			putc(c);
	};

/**
 *  @brief   Return number of bytes waiting in the receive buffer
 *  @param   none
 *  @return  bytes waiting in the receive buffer
 */
	static int available(void) {
		return (RX_BUFFER_MASK + RxHead - RxTail) % RX_BUFFER_MASK;
	};


/**
 *  @brief   Flush bytes waiting in receive buffer
 *  @param   none
 *  @return  none
 */
	inline static void flush(void) { 
		RxHead = RxTail;
	};

	inline static void transmit_handler() {
		if ( TxHead != TxTail) {
			unsigned char tmptail;
			/* calculate and store new buffer index */
			tmptail = (TxTail + 1) & TX_BUFFER_MASK;
			TxTail = tmptail;
			/* get one byte from buffer and write it to UART */
			_SFR_MEM8(DATA_REG) = TxBuf[tmptail];  /* start transmission */
		}else{
			/* tx buffer empty, disable UDRE interrupt */
			_SFR_MEM8(CONTROL_REG) &= ~_BV(INT_PIN);
		}
	};
	
	inline static void receive_handler() {
		unsigned char tmphead;
		unsigned char data;
		unsigned char usr;
		unsigned char lastRxError;
		    
		    
		/* read UART status register and UART data register */
		usr  = _SFR_MEM8(STATUS_REG);
		data = _SFR_MEM8(DATA_REG);
		lastRxError = usr & (ERROR_MASK);    
		tmphead = ( RxHead + 1) & RX_BUFFER_MASK;
		    
		if ( tmphead == RxTail ) {
			/* error: receive buffer overflow */
			lastRxError = BUFFER_OVERFLOW >> 8;
		}else{
			/* store new index */
			RxHead = tmphead;
			/* store received data in buffer */
			RxBuf[tmphead] = data;
		}
		LastRxError = lastRxError;
	};

}; // end uart template class 

template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::TxBuf[TX_BUFFER_SIZE];
template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::RxBuf[RX_BUFFER_SIZE];
template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::TxHead = 0;
template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::TxTail = 0;
template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::RxHead = 0;
template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::RxTail = 0;
template < uint16_t STATUS_REG, uint16_t CONTROL_REG, uint8_t CONTROL_MASK, uint16_t CONTROL2_REG, uint8_t CONTROL2_MASK, uint16_t DATA_REG, uint8_t INT_PIN, uint8_t ERROR_MASK, uint16_t BAUDRATE_REG_H, uint16_t BAUDRATE_REG_L, uint8_t DOUBLE_SPEED_PIN >
volatile unsigned char uart< STATUS_REG, CONTROL_REG, CONTROL_MASK, CONTROL2_REG, CONTROL2_MASK, DATA_REG, INT_PIN, ERROR_MASK, BAUDRATE_REG_H, BAUDRATE_REG_L, DOUBLE_SPEED_PIN >::LastRxError = 0;

/**@}*/


#endif // _UART_H_

