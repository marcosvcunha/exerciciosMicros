;
; Contador-Display7.asm
;
; Created: 08/09/2019 17:43:01
; Author : Marcos Cunha
;


; Replace with your application code

;; R9 contem o contador
; Colocando os dados na mesmoria:

	ldi R16, 0xFC
	ldi R17, 0x90
	ldi R18, 0x7A
	ldi R19, 0xDA
	ldi R20, 0x96
	ldi R20, 0xCE
	ldi R20, 0xEE
	ldi R20, 0x98
	ldi R20, 0xFE
	ldi R20, 0xDE

start:
	;; Colocar as portas B e D para saida
    ldi R20, 0
;; Substituir count por um botão que incrementa R9
count:
	rcall display
	inc r9
	rjmp count

display:


	ret