;
; i2c_lcd_control.asm
;
; Created: 10/30/2018 10:44:25 AM
; Author : jlfragoso
;

.dseg
adc_data: .byte 2
adc_asc: .byte 10
; Replace with your application code
.cseg
.org 0x0000
;ISR List vector
jmp start  ; reset
.org 0x0024
jmp usart_receiver_isr
.org 0x002a
jmp adc_complete ; adc conversion end

.org INT_VECTORS_SIZE
.include "lcd_i2c.inc"
.include "usart.inc"

start:
	ldi r16, high(RAMEND) 
	out sph, r16
	ldi r16, low(RAMEND)
	out spl, r16
	eor r1,r1
	ldi r24, low(12345)
	ldi r25, high(12345)
	rcall int16_to_asc


	rcall buffer_flush
	rcall usart_init
	rcall twi_init
	rcall lcd_init
	eor r5, r5
	eor r7, r7

	ldi zl, low(_msg1)
	ldi zh, high(_msg1)
	rcall lcd_write_str_flash
_go_again: 
	lcd_set_cursor 1,0
	ldi zl, low(_msg2)
	ldi zh, high(_msg2)
	rcall lcd_write_str_flash 

	wait_button_press_and_release PIND, PD4
	;; configurar o timer 1 
	ldi r24, low(3125)
	ldi r25, high(3125)
	sts OCR1AH, r25
	sts OCR1AL, r24
	sts OCR1BH, r25
	sts OCR1BL, r24
	ldi r24, 0x00
	sts TCNT1H, r24
	sts TCNT1L, r24
	sts TIMSK1, r24 ; SEM INTERRUPCAO DO TIMER
	sts TCCR1A, r24
	ldi r24, (1<<WGM12) | (5 << CS10) ; modo 4 e prescaler 1024
	sts TCCR1B, r24


	ldi r24, (1<<ADC0D) ; desabilita digital
	sts DIDR0, r24
	ldi r24, (1<<REFS0) ; usar avcc e ADC0
	sts ADMUX, r24
	ldi r24, (5 << ADTS0)
	sts ADCSRB, r24 ; free running
	ldi r24, (1<<ADEN) | (1<<ADSC)| (1<<ADATE) | (1<<ADIE) |(1<<ADIF) | (7<<ADPS0)
	sts ADCSRA, r24 ;; comeca conversao
	sei

;espera_conv:
;	lds r24, ADCSRA
;	andi r24, (1<<ADIF)
;	breq espera_conv
	rcall lcd_clear
	ldi zl, low(_msg3)
	ldi zh, high(_msg3)
	rcall lcd_write_str_flash 

	; conversao acabou

_loop_update:
	;; print ADC
	;lds r24,(adc_data+1)
	;rcall lcd_write_reg_hex
	;lds r24, adc_data
	;rcall lcd_write_reg_hex
	;write_lcd_data ' '
	;write_lcd_data_reg r7

	lds r24, (adc_data)
	lds r25, (adc_data+1)
	ldi xl, low(adc_asc)
	ldi xh, high(adc_asc)
	rcall int16_to_asc
	ldi zl, low(adc_asc)
	ldi zh, high(adc_asc) 
	rcall lcd_write_str_sram
	;;imprime buffer
_print_buffer:
	lds r24, (_buffer_flag)
	tst r24
	breq _loop_continue
	ldi r24, 1
	mov r25, r5
	rcall _lcd_set_cursor
	rcall buffer_read
	write_lcd_data_reg r18
	inc r5
	rjmp _print_buffer

_loop_continue:
	lcd_set_cursor 0,6

fim:
	rjmp _loop_update

adc_complete:
	lds r16, ADCL
	lds r17, ADCH
	sts adc_data, r16
	sts (adc_data+1), r17
	ldi r16, 0xff
	out TIFR1, r16 ; clear timer/counter 1 flags
	reti


_constants:
_msg1: .db "Hello Uergs!",0,0
_msg2: .db "Press button",0,0
_msg3: .db "ADC = ",0,0
