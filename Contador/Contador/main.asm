;
; Contador.asm
;
; Created: 08/09/2019 17:21:50
; Author : CLIENTE
;


; Replace with your application code
start:
    ldi r17, 0
loop0:
	ldi r16, 0
loop1:
	dec r16
	brne loop1
	dec r17
	rjmp loop0
