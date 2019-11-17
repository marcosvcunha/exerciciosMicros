;
; Prova Q6.asm
;
; Created: 07/10/2019 16:21:12
; Author : Marcos
;

.cseg
.org 0x0000
	jmp start					; 0x0000 Reset

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
	sts ICR1H, r16
	sts ICR1L, r17
	ldi r16, 0x80 ;; cooloca 128 
	sts OCR1BL, r16
    ldi R16, (0x2 << COM0A0) | (0x0E << WGM10)
	out TCCR0A, R16
	ldi R16, 0x02 
	sts TCCR1B, R16 ; PRESCALE

fim:
	rjmp fim
