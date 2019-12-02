;
; medidor_capacitancia.asm
;
; Created: 25/11/2019 15:14:12
; Author : CLIENTE
;
.dseg
value: .byte 10
; Replace with your application code
.cseg
.org 0x0000
;ISR List vector
jmp start  ; reset
.org 0x0014
jmp ISR_capture
.org 0x001a
jmp ISR_capture

.org INT_VECTORS_SIZE
.include "lcd_i2c.inc"


; Replace with your application code
start:
	ldi r16, high(RAMEND) 
	out sph, r16
	ldi r16, low(RAMEND)
	out spl, r16
	eor r1, r1
	
	rcall twi_init
	rcall lcd_init
	
	ldi r16, (1<<2)
	out DDRD, r16
	ldi r16, 0
	out PORTD, r16
	ldi r16, (1<<0)|(1<<1)
	out DDRB, r16
	ldi r16, 2
	out PORTB, r16

	ldi zl, low(_msg1)
	ldi zh, high(_msg1)
	rcall lcd_write_str_flash
	lcd_set_cursor 1,0
	ldi zl, low(_msg2)
	ldi zh, high(_msg2)
	rcall lcd_write_str_flash 
	wait_button_press_and_release PIND, PD3
	rcall lcd_clear
	write_lcd_data '.'
	ldi r16, (1<<ACBG)|(1<<ACIC)|(3<<ACIS0)
	out ACSR, r16
	ldi r16, (1<<AIN1D)
	sts DIDR1, r16


	ldi r16, (1<<ICIE1)|(1<<TOIE1)
	sts TIMSK1, r16
loop_medida:
	eor r1,r1
	eor r7,r7
	eor r8, r8
	eor r9, r9
	sei
	sts TCNT1H, r1
	sts TCNT1L, r1
	ldi r16, 0xff
	out TIFR1, r16
	ldi r16, (1<<ICNC1) | (3<<CS10)
	sts TCCR1B, r16

	sbi PORTD, PD2
	ldi r16, 2
	out PORTB, r16
espera:
	tst r7
	breq espera
conv:
	cli
	cbi PORTD, PD2
	ldi r16, 0
	sts TCCR1B, r16 ; stop counter

	rcall lcd_clear
	lcd_set_cursor 0,0
		mov r24, r8
	mov r25, r9
	ldi xl, low(value)
	ldi xh, high(value)
	rcall int16_to_asc

	ldi zl, low(value)
	ldi zh, high(value) 
	rcall lcd_write_str_sram

	lcd_set_cursor 1,0
	ldi zl, low(_msg3)
	ldi zh, high(_msg3)
	rcall lcd_write_str_flash 
	cbi PORTD, 2

	mov r16, r8
	mov r17, r9
	eor r10, r10
	ldi r18, 7
	mul r16, r18
	mov r8, r0
	mov r9, r1
	mul r17, r18
	add r9, r0
	adc r10, r1

	clc
	ror r10
	ror r9
	ror r8
	clc
	ror r10
	ror r9
	ror r8
	clc
	mov r24, r8
	mov r25, r9
	ldi xl, low(value)
	ldi xh, high(value)
	rcall int16_to_asc

	ldi zl, low(value)
	ldi zh, high(value) 
	rcall lcd_write_str_sram
fim:
	write_lcd_data 'n'
	write_lcd_data 'F'

	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
	rjmp loop_medida

ISR_capture:
	lds r8, ICR1L
	lds r9, ICR1H
	inc r7
	reti

_constants:
_msg1: .db "CapxMeter!",0,0
_msg2: .db "Press button",0,0
_msg3: .db "Medido= ",0,0
