
; LCD_Test.asm
;
; Created: 9/11/2017 9:15:52 AM
; Author : jlfragoso
;
.cseg
.org 0x0000
;ISR Lists
	jmp start					; 0x0000 Reset
;	jmp	irq0 					; 0x0002 IRQ0
;	jmp INT0_ 					; 0x0004 IRQ1
;	jmp PCINT1_					; 0x0006 PCINT0
;	jmp PCINT1_					; 0x0008 PCINT1
;	jmp PCINT2 					; 0x000A PCINT2
;	jmp WDT 					; 0x000C Estouro de Watchdog
;	jmp TIM2_COMPA 				; 0x000E Timer2 Comparacao A
;	jmp TIM2_COMPB 				; 0x0010 Timer2 Comparacao B
;	jmp TIM2_OVF 				; 0x0012 Timer2 Estouro
;	jmp TIM1_CAPT 				; 0x0014 Timer1 Captura
;	jmp TIM1_COMPA 				; 0x0016 Timer1 Comparacao A
;	jmp TIM1_COMPB 				; 0x0018 Timer1 Comparacao B
; 
;	jmp TIM1_OVF 				; 0x001A Timer1 Estouro
;	jmp TIM0_COMPA 				; 0x001C Timer0 Comparacao A
;	jmp TIM0_COMPB 				; 0x001E Timer0 Comparacao B

;	.org 0x0020
;	jmp TIM0_OVF 				; 0x0020 Timer0 Estouro
;	jmp SPI_STC 				; 0x0022 SPI Transferencia Completa
	.org 0x0024
	jmp USART_RXC 				; 0x0024 USART RX Completa
;	jmp USART_EMPTY				; 0x0026 Registro de Dados Vazio na USART
;	jmp USART_TXC				; 0x0028 USART TX Completa
;	jmp ADC_COMP                ; 0x002A ADC Conversion Complete
;	jmp EE_READY				; 0x002C EEPROM Ready
;	jmp ANALOG_COMP				; 0x002E Analog Comparator
;   jmp TWI						; 0x0030 2-wire Serial Interrupt (I2C)
;	jmp SPM_READY				; 0x0032 Store Program Memory Ready

.org INT_VECTORS_SIZE
start:
	call USART_Init
	sbi DDRD, 2
	ldi r18, 0



loop:
	call USART_Receive
	rjmp loop


USART_Receive:
; Espera buffer RX disponivel
	lds r17, UCSR0A
	sbrs r17, RXC0
	rjmp USART_Receive
	lds r16, PORTD
	neg r16
	out PORTD, r16
; Lê dado (r16) do buffer
	lds r16, UDR0
	ret



USART_Init:
; Definir taxa de transmissão UBRR0
	ldi r16, 16
	ldi r17, 0
	sts UBRR0H, r17
	sts UBRR0L, r16
; Habilitar RX e TX
	ldi r16, (1<<RXEN0)|(1<<TXEN0)
	sts UCSR0B,r16
; Defini quadro 8 bits dado, 2 stop bit
	ldi r16, (1<<USBS0)|(3<<UCSZ00)
	sts UCSR0C,r16
	ret

USART_Init2:
; Definir taxa de transmissão UBRR0
	ldi r16, 16
	ldi r17, 0
	sts UBRR0H, r17
	sts UBRR0L, r16
	ldi r16, (1 << RXC0)
	sts UCSR0B, r16
; Habilitar RX e TX
	ldi r16, (1<<RXEN0)|(1<<TXEN0)|(1 << RXCIE0)
	sts UCSR0B,r16
; Defini quadro 8 bits dado, 2 stop bit
	ldi r16, (1<<USBS0)|(3<<UCSZ00)
	sts UCSR0C,r16
	sei
	ret

USART_RXC:
	in r17, SREG
	lds r16, UDR0
	sbi PORTD, 2
	rjmp usart_end
	ldi r19, 0
	cp r18, r19
	breq liga
	ldi r18, 0
	cbi PORTD, 2
	rjmp usart_end
liga:
	ldi r18, 1
	sbi PORTD, 2
usart_end:
	out SREG, r17 
	reti