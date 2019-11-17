;
; AssemblerApplication1.asm
;
; Created: 08/09/2019 17:35:32
; Author : Marcos Cunha
;


; Replace with your application code
start:
    ldi R16, 0xFF
	out DDRB, R16
loop:
	sbi PORTB, 5
	rcall delay_05
	rcall delay_05
	rcall delay_05
	rcall delay_05
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