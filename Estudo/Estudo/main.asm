;
; Estudo.asm
;
; Created: 06/10/2019 17:16:42
; Author : Marcos Cunha
;


; Replace with your application code
start:
	ldi R16, 0xFF
	out DDRB, R16 ;
	out PORTB, R16
loop:
	rjmp loop
