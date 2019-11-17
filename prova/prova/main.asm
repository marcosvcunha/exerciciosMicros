;
; prova.asm
;
; Created: 07/10/2019 14:00:20
; Author : CLIENTE
;


; Replace with your application code
start:
	ldi r16, 0xff
	ldi r17, 0
	dec r16
	inc r17
	brne start
fim:inc r16
	rjmp fim