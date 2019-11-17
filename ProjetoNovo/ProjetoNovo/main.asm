;
; ProjetoNovo.asm
;
; Created: 09/09/2019 13:49:36
; Author : CLIENTE
;

.cseg
.org 0x0000
;ISR Lists
	jmp start					; 0x0000 Reset
	jmp	irq0 					; 0x0002 IRQ0
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

	.org 0x0020
	jmp TIM0_OVF 				; 0x0020 Timer0 Estouro
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


.include "display7.inc"
.equ BUTTON = PIND
.equ BUTTON_BIT = 2
; Replace with your application code
start:
	; configuracao da pilha
	ldi r16, 1
	mov r5,r16
	ldi r16, high(RAMEND)   
	out sph, r16
	ldi r16, low(RAMEND)
	out spl, r16

	; configura registrador 
	ldi R16,0x01
	sts TIMSK0, R16
	ldi R16, 0x00 ;; eu nao lembro exatamente oq é que tem que usar
	out TCCR0A, R16
	ldi R16, 0x05
	out TCCR0B, R16 ; PRESCALE
	; Registradores e porta B
	ldi R16, 38
	mov R4, R16 ; 38 INTERRUPÇOES 
	sbi DDRB, 5 ; porta para saída
	cbi PORTB, 5
	
	; zerando r1
	clr r1
	; inicializacao do display
	cbi BUTTON+1, BUTTON_BIT ; seta DDR
	sbi BUTTON+2, BUTTON_BIT;; ligou pull-up 
	rcall init_display
	ldi r16, 0x0F 
	; habilitando interrupçao
	ldi r19, (1<<int0); ???
	sts EICRA, r19
	out EIMSK, r19
	sei
loop:
	mov r24, r16
	ldi r25, 0
	rcall display_value
	rcall delay_05
	add r16, r5
	ldi r17, 0x10
	cp r16, r17
	brne pc+2
	ldi r16, 0
	ldi r17, 0xFF
	cp r16, r17
	brne pc+2
	ldi r16, 0x0F
	rjmp loop

delay_05:
  ldi R18, 20
loop1:
  ldi R24, low(3037)
  ldi R25, high(3037)
delay_loop:
  adiw R24, 1
  brne delay_loop
  dec R18
  brne loop1
  ret

irq0:
	neg r5
	reti 

TIM0_OVF: 
	ldi R16, 0x00
	DEC R4
	cp R4, R16
	breQ fim_time
	reti
fim_time:
	ldi R16, 38
	mov R4, R16 ; 38 INTERRUPÇOES 
	in R16, PORTB
	ldi R20, (1<<5)
	eor R16, R20
	out PORTB, R16
    RETI
