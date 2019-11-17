;
; Prova Q7.asm
;
; Created: 07/10/2019 16:24:33
; Author : CLIENTE
;

; D2 botao
; D3 led verde
; D4 led amarelo
; D5 led vermelho

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
	;ldi r16, low(0x7FFF)
	;sts OCR1AH, r16
	;ldi r16, high(0xFFFF)
	;sts OCR1AL, r16
	;ldi r16, 0x05; prescale
	;sts TCCR1B, r16
	;ldi r16, (0x01 << COM1A0 | 0x04 << WGM10)
	;sts TCCR1A, r16
	;ldi r16, 2
	;sts TIMSK1, r16
	;; habilitando interrupçao
	cbi DDRD, 2
	ldi r19, (1<<int0)
	sts EICRA, r19
	out EIMSK, r19
	sei

	; liga verde
	sbi PORTD, 3
loop:
	ldi r19, 0
	cp r20, r19 ; R20 tem a flag que é ativa pelo botao
	breq loop
	;delay 20
	call delay_05
	call delay_05
	;desliga verde
	cbi PORTD, 3	
	;liga amarelo
	sbi PORTD, 4
	;delay 10
	call delay_05
	;desliga amarelo
	cbi PORTD, 4
	;liga vermelho
	SBI PORTD, 5
	;delay 60
	call delay_05
	call delay_05
	call delay_05
	call delay_05
	call delay_05
	call delay_05
	;clear r20
	ldi r20, 0
	;desliga vermelho
	cbi PORTD, 5
	;liga verde
	SBI PORTD, 3
	;delay 40
	call delay_05
	call delay_05
	call delay_05
	call delay_05
	rjmp loop

irq0:
	;set R20
	ldi r20, 0xFF
	reti

delay_05:
  	ldi R18, 40
loop1:
  	ldi R24, low(3037)
  	ldi R25, high(3037)
delay_loop:
  	adiw R24, 1
  	brne delay_loop
  	dec R18
  	brne loop1
  	ret
