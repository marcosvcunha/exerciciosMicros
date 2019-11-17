;
; LCD_Test.asm
;
; Created: 9/11/2017 9:15:52 AM
; Author : jlfragoso
;
.cseg
.org 0x0000
;ISR Lists
	jmp start					; 0x0000 Reset
.org 0x002A
	jmp ADC_COMP                ; 0x002A ADC Conversion Complete

.macro delay_us
	ldi r24, @0
	rcall _delay_us
.endm

.macro delay_6ms
	rcall delay_2ms
	rcall delay_2ms
	rcall delay_2ms
.endm

; Replace with your application code
start:
	sbi DDRD, 2
conf_conv:
	ldi r16, (1 << REFS0) | (1 << ADLAR) | (0 << MUX0) ; 0 - 7 escolhe entrada
	sts ADMUX, r16
	ldi r16, 1
	sts DIDR0, r16 ; desabilita entrada digital 0 da porta C
	ldi r16, (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | (7 << ADPS0)
	sts ADCSRA, r16
	sei
start_conv:
	ldi r16, (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (1 << ADIE) | (7 << ADPS0)
	sts ADCSRA, r16

loop:
	rjmp loop

delay_1us:
	nop
	ldi r24, 2
loop1_d1:
	dec r24
	brne loop1_d1
	ret

_delay_us:  ; carregar em r24 valor em us
	ldi r25, 3
loop1_d45:
	nop
	dec r25
	brne loop1_d45
	dec r24
	brne _delay_us
	ret

delay_2ms:
	ldi r24, low(57520)
	ldi r25, high(57520)
loop1_d2:
	adiw r24,1
	brne loop1_d2
	ret

ADC_COMP:
	lds r16, ADCH
	ldi r17, 0xEF
	cp r17, r16
	brge end
	sbi PIND, 2
end:
	ldi r16, (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (1 << ADIE) | (7 << ADPS0)
	sts ADCSRA, r16
	reti
less:
	cbi PIND, 2
	rjmp end
	