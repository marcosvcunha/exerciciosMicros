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
	.org 0x0020
	jmp TIM1_COMPA 				; 0x0016 Timer1 Comparacao A
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
.org INT_VECTORS_SIZE

start:
; =========================== configuracao da pilha ============================
	ldi r16, 1
	mov r5,r16
	ldi r16, high(RAMEND)   
	out sph, r16
	ldi r16, low(RAMEND)
	out spl, r16
; ============================== setando timer =================================
	ldi r16, low(624)
	sts OCR1AL, r16
	ldi r16, high(624)
	sts OCR1AH, r16
	ldi r16, (0x01 << WGM12 | 0x04 << CS10); prescale
	sts TCCR1B, r16
	ldi r16, (0x00 << COM1A0 | 0x00 << WGM10)
	sts TCCR1A, r16
	ldi r16, 2
	sts TIMSK1, r16
; ============================= setando Registradores ==========================
	sbi DDRD, 2
	ldi r17, 0
	ldi r18, 0
	ldi r19, 0
	sei

loop:
	rjmp loop

TIM1_COMPA: 
	;--------R7-----------
	inc R7
	;Checando os centesimos de segundo dos bits menos significativos do registrador
	ldi R16, 0x0f ;
	and R16, R7 
	subi R16, 0x0a
	brne interrupt_end;se nao chegar a 10, pula pro fim da interrupção
	;limpando
	ldi R16, 0xf0
	and R7, R16
	;incrementando
	ldi R16, 0x10
	add R7, R16 ;incrementa os bits mais significativos do R7

	;Checando os centesimos de segundo dos bits mais significativos do registrador
	ldi R16, 0xf0 
	and R16, R7
	subi R16, 0xa0
	brne interrupt_end;se nao chegar a 10, pula pro fim da interrupção
	;limpando
	ldi R16, 0x0f
	and R7, R16

	neg r19
	breq aqui
	cbi PORTD,2
	rjmp aqui2
aqui:
	sbi PORTD, 2
aqui2:
	;---------R8------------
	inc R8; incrementa o contador de segundos
	;Checando os bits menos significativos do registrador de segundos
	ldi R16, 0x0f
	and R16, R8
	subi R16, 0x0a
	brne interrupt_end;se nao chegar a 10, pula pro fim da interrupção
	;limpando
	ldi R16, 0xf0
	and R8, R16
	;incrementando
	ldi R16, 0x10
	add R8, R16; incrementa os bits mais significativos do  R8

	;Checando os segundos dos bits mais significativos do registrador
	ldi R16, 0xf0 
	and R16, R8
	subi R16, 0x60
	brne interrupt_end;se nao chegar a 6, pula pro fim da interrupção
	;limpando
	ldi R16, 0x0f
	and R8, R16

	;--------R9-------------
	inc R9; incrementa o contador de segundos
	;Checando os bits menos significativos do registrador de minutos
	ldi R16, 0x0f
	and R16, R9
	subi R16, 0x0a
	brne interrupt_end;se nao chegar a 10, pula pro fim da interrupção
	;limpando
	ldi R16, 0xf0
	and R9, R16

	;incrementando
	ldi R16, 0x10
	add R9, R16; incrementa os bits mais significativos do  R8

	;Checando os minutos dos bits mais significativos do registrador
	ldi R16, 0xf0 
	and R16, R9
	subi R16, 0x60
	brne interrupt_end;se nao chegar a 6, pula pro fim da interrupção
	;limpando
	ldi R16, 0x0f
	and R9, R16
	
	;---------R10-----------
	inc R10
	;Checando as horas dos bits menos significativos do registrador
	ldi R16, 0x0f ;
	and R16, R10
	subi R16, 0x0a
	brne interrupt_end;se nao chegar a 10, pula pro fim da interrupção
	;limpando
	ldi R16, 0xf0
	and R10, R16
	;incrementando
	ldi R16, 0x10
	add R10, R16 ;incrementa os bits mais significativos do R7

	;Checando as horas dos bits mais significativos do registrador
	ldi R16, 0xf0 
	and R16, R10
	subi R16, 0xa0
	brne interrupt_end;se nao chegar a 10, pula pro fim da interrupção
	;limpando
	ldi R16, 0x0f
	and R10, R16
interrupt_end:
	RETI