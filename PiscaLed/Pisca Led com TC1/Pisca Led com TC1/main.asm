;
; Motor.asm
;
; Created: 23/09/2019 14:31:20
; Author : CLIENTE
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
;	jmp TIM1_OVF 				; 0x001A Timer1 Estouro
;	jmp TIM0_COMPA 				; 0x001C Timer0 Comparacao A
;	jmp TIM0_COMPB 				; 0x001E Timer0 Comparacao B

;	.org 0x0020
;	jmp TIM0_OVF 				; 0x0020 Timer0 Estouro
;	jmp SPI_STC 				; 0x0022 SPI Transferencia Completa
;	jmp USART_RXC 				; 0x0024 USART RX Completa
;	jmp USART_EMPTY				; 0x0026 Registro de Dados Vazio na USART
;	jmp USART_TXC				; 0x0028 USART TX Completa
;	jmp ADC_COMP                ; 0x002A ADC Conversion Complete
;	jmp EE_READY				; 0x002C EEPROM Ready
;	jmp ANALOG_COMP				; 0x002E Analog Comparator
;   jmp TWI						; 0x0030 2-wire Serial Interrupt (I2C)
;	jmp SPM_READY				; 0x0032 Store Program Memory Ready




; Replace with your application code
.org INT_VECTORS_SIZE
.equ BUTTON = PIND
.equ BUTTON_BIT = 2


start:
	; configuracao da pilha
	ldi r16, 1
	mov r5,r16
	ldi r16, high(RAMEND)   
	out sph, r16
	ldi r16, low(RAMEND)
	out spl, r16

	; configura registrador 
	sbi DDRB, 1
	sbi DDRD, 2
	;; -- setando a comparaçao
	ldi r16, high(62499)
	sts OCR1AH, r16
	ldi r16, low(62499)
	sts OCR1AL, r16
    ldi R16, (0x01 << COM1A0) | (0x04 << WGM10)
	sts TCCR1A, R16
	ldi R16, 0x04 
	sts TCCR1B, R16 ; PRESCALE
	;sbi PORTD, 2

	; Registradores e porta B

	;cbi BUTTON+1, BUTTON_BIT ; seta DDR
	;sbi BUTTON+2, BUTTON_BIT;; ligou pull-up 
	;; habilitando interrupçao
	;ldi r19, (1<<int0); ???
	;sts EICRA, r19
	;out EIMSK, r19
	;sei

fim:
	in r17, PORTB
	ROL r17
	;sts PORTD2, R17
	out PORTD, r17
	rjmp fim

irq0:
	ldi r17, 0xFF
	ldi r18, 0x7F
	ldi r19, 0x3F
	ldi r20, 0x00

	cp r16, r17
	BREQ set0
	cp r16, r18
	BREQ set255
	cp r16, r19
	BREQ set127
	cp r16, r20
	BREQ set63
set0:;; aqui ele coloca R16 no proximo valor
	mov r16, r20
	rjmp aqui 
set63:
	mov r16, r19
	rjmp aqui
set127:
	mov r16, r18
	rjmp aqui
set255:
	mov r16, r17
	rjmp aqui
aqui:
	out OCR0A, r16 ;;
	reti 