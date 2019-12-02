/*
 * usart.inc
 *
 *  Created: 11/13/2018 11:07:15 AM
 *   Author: jlfragoso
 */ 

 #ifndef __USART_INC__
 #define __USART_INC__


 usart_init:
	ldi r16, 16
	ldi r17, 0
	sts UBRR0H, r17
	sts UBRR0L, r16
	ldi r16, (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0) 
	sts UCSR0B, r16
	ldi r16, (1<<USBS0) | (3<<UCSZ00)
	sts UCSR0C, r16
	ldi r16, (1<<U2X0)
	sts UCSR0A, r16
	ret

usart_receiver_isr:
	lds r18, UDR0
	mov r7, r18
	rcall buffer_write
	reti

usart_transmit:
	lds r17, UCSR0A
	sbrs r17, UDRE0
	rjmp usart_transmit
	sts UDR0, r16
	ret
 #endif


