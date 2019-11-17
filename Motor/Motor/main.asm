
; Created: 23/09/2019 14:31:20
; Author : CLIENTE
;

.cseg
.org 0x0000
	jmp start					; 0x0000 Reset
	jmp	irq0 					; 0x0002 IRQ0

.org INT_VECTORS_SIZE
.equ BUTTON = PIND
.equ BUTTON_BIT = 2


start:
	ldi r16, high(RAMEND)   
	out sph, r16
	ldi r16, low(RAMEND)
	out spl, r16

	; configura registrador 
	ldi R16,0x04  ; ativa comp B
	sts TIMSK1, R16
	ldi r16, high(0x01FF)
	ldi r17, low(0x01FF)
	out ICR1H, r16
	out ICR1L, r17
	ldi r16, 0x80 ;; cooloca 128 
	out OCR1BL, r16
    ldi R16, (0x2 << COM0A0) | (0x0E << WGM10)
	out TCCR0A, R16
	ldi R16, 0x02 
	out TCCR1B, R16 ; PRESCALE

fim:
	rjmp fim
