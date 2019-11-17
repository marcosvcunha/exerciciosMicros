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


;start:
;	; configuracao da pilha
;	ldi r16, 1
;	mov r5,r16
;	ldi r16, high(RAMEND)   
;	out sph, r16
;	ldi r16, low(RAMEND)
;	out spl, r16
;	sbi DDRD, 3
;	sbi PORTD, 3
;	sbi PIND, 3

start:
ldi R16, 0xFF
out DDRB, R16 ;
loop:
sbi PORTB, 5
rcall delay_05
cbi PORTB, 5
rcall delay_05
rjmp loop

delay_05:
ldi R16, 8
loop1:
ldi R24, low(3037)
ldi R25, high(3037)
delay_loop:
adiw R24, 1
brne delay_loop
dec R16
brne loop1
ret