;
; Prova Q7.asm
;
; Created: 07/10/2019 16:24:33
; Author : CLIENTE
;

.cseg
.org 0x0000
	jmp start					; 0x0000 Reset
	jmp	irq0 					; 0x0002 IRQ0



; Replace with your application code
.org INT_VECTORS_SIZE
; Replace with your application code
start:
	sbi DDRD, 3
	sbi DDRD, 4
	sbi DDRD, 5
	; setando timer
	ldi r16, low(0x7FFF)
	sts OCR1AH, r16
	ldi r16, high(0xFFFF)
	sts OCR1AL, r16
	ldi r16, 0x05; prescale
	sts TCCR1B, r16
	ldi r16, (0x01 << COM1A0 | 0x04 << WGM10)
	sts TCCR1A, r16
	ldi r16, 2
	sts TIMSK1, r16
	;; habilitando interrupçao
	cbi DDRD, 2
	ldi r19, (1<<int0)
	sts EICRA, r19
	out EIMSK, r19
	sei
loop:
	ldi r19, 0
	cp r20, r19 ; R20 tem a flag que é ativa pelo botao
	breq start
	;delay 20
	;desliga verde
	;liga amarelo
	;delay 10
	;desliga amarelo
	;liga vermelho
	;delay 60
	;clear r20
	;desliga vermelho
	;liga verde
	;delay 40
	rjmp loop

irq0:
	sbi PORTD, 3
	reti