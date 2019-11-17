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
;	jmp	irq0 					; 0x0002 IRQ0
;	jmp INT0_ 					; 0x0004 IRQ1
;	jmp PCINT1_					; 0x0006 PCINT0
;	jmp PCINT1_					; 0x0008 PCINT1
;	jmp PCINT2 					; 0x000A PCINT2
;	jmp WDT 					; 0x000C Estouro de Watchdog
;	jmp TIM2_COMPA 				; 0x000E Timer2 Comparacao A
;	jmp TIM2_COMPB 				; 0x0010 Timer2 Comparacao B
;	jmp TIM2_OVF 				; 0x0012 Timer2 Estouro
;	jmp TIM1_CAPT 				; 0x0014 Timer1 Captura
;	jmp TIM1_COMPA 				; 0x0016 Timer1 Comparacao A
;	jmp TIM1_COMPB 				; 0x0018 Timer1 Comparacao B
;	jmp TIM1_OVF 				; 0x001A Timer1 Estouro
;	jmp TIM0_COMPA 				; 0x001C Timer0 Comparacao A
;	jmp TIM0_COMPB 				; 0x001E Timer0 Comparacao B

;	.org 0x0020
;	jmp TIM0_OVF 				; 0x0020 Timer0 Estouro
;	jmp SPI_STC 				; 0x0022 SPI Transferencia Completa
;	jmp USART_RXC 				; 0x0024 USART RX Completa
;	jmp USART_EMPTY				; 0x0026 Registro de Dados Vazio na USART
;	jmp USART_TXC				; 0x0028 USART TX Completa
.org 0x002A
	jmp ADC_COMP                ; 0x002A ADC Conversion Complete
;	jmp EE_READY				; 0x002C EEPROM Ready
;	jmp ANALOG_COMP				; 0x002E Analog Comparator
;   jmp TWI						; 0x0030 2-wire Serial Interrupt (I2C)
;	jmp SPM_READY				; 0x0032 Store Program Memory Ready


.equ LCD_CTRL = PORTB
.equ LCD_EN = PB0
.equ LCD_RS = PB1
.equ LCD_DATA = PORTD
.equ LCD_POWER_UP = 0x30
.equ LCD_8BIT_2LINES = 0x38 
.equ LCD_8BIT_1LINE  = 0x30  ; power_up
.equ LCD_4BIT_2LINES = 0x28 
.equ LCD_4BIT_1LINE  = 0x20 
.equ LCD_DON_COFF_BOFF = 0x0C  ;--display on, cursor off, blink off
.equ LCD_DON_COFF_BON = 0x0D  ;--display on, cursor off, blink on
.equ LCD_DON_CON_BOFF = 0x0E  ;--display on, cursor on, blink off
.equ LCD_DON_CON_BON = 0x0F  ;--display on, cursor on, blink on
.equ LCD_DOFF_COFF_BOFF = 0x08  ;--display off, cursor off, blink off
.equ LCD_DOFF_COFF_BON = 0x09  ;--display off, cursor off, blink on
.equ LCD_DOFF_CON_BOFF = 0x0A  ;--display off, cursor on, blink off
.equ LCD_DOFF_CON_BON = 0x0B  ;--display off, cursor on, blink on
.equ LCD_MODE_INC = 0x06
.equ LCD_MODE_INC_SHIFT = 0x07
.equ LCD_MODE_DEC = 0x04
.equ LCD_MODE_DEC_SHIF = 0x05
.equ LCD_CLEAR = 0x01

.macro delay_us
	ldi r24, @0
	rcall _delay_us
.endm

.macro delay_6ms
	rcall delay_2ms
	rcall delay_2ms
	rcall delay_2ms
.endm

.macro write_lcd_cmd
	ldi r24, @0
	rcall _write_lcd_cmd
	delay_us 50
.endm

.macro write_lcd_data
	ldi r24, @0
	rcall _write_lcd_data
	delay_us 50
.endm

.macro write_lcd_data_reg
	mov r24, @0
	rcall _write_lcd_data
	delay_us 50
.endm

; Replace with your application code
start:
	ldi r24,0xff
	out ddrd, r24
	ldi r24, 0x03
	out ddrb, r24
	rcall init_lcd
	write_lcd_data 'h';
	write_lcd_data 'e';
	write_lcd_data 'l';
	write_lcd_data 'l';
	write_lcd_data 'o';
	;write_lcd_cmd 0xC0
	rcall _delay_us
conf_conv:
	ldi r16, (1 << REFS0) | (1 << ADLAR) | (0 << MUX0) ; 0 - 7 escolhe entrada
	sts ADMUX, r16
	ldi r16, 1
	sts DIDR0, r16 ; desabilita entrada digital 0 da porta C
	ldi r16, (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | (7 << ADPS0)
	sts ADCSRA, r16
	
	ldi r18, 0x00 ; valor atual mostrado no display
	ldi r19, 0x00 
start_conv:
	ldi r16, (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (1 << ADIE) | (7 << ADPS0)
	sts ADCSRA, r16
	sei
loop:
	cp r18, r19
	breq loop
	mov r18, r19
	write_lcd_cmd 0x88

	mov r16, r18
	com r16
	swap r16
	ldi r17, 0x0f
	and r16, r17
	ori r16, 0x30
	ldi r17, 0x3a
	cp r16, r17
	brlo show_h
	ldi r17, 7
	add r16, r17
show_h:
	write_lcd_data_reg r16

	mov r16, r18
	com r16
	ldi r17, 0x0f
	and r16, r17
	ori r16, 0x30
	ldi r17, 0x3a
	cp r16, r17
	brlo show_l
	ldi r17, 7
	add r16, r17
show_l:
	write_lcd_data_reg r16

	write_lcd_cmd 0xC0
	mov r16, r18
	com r16
	swap r16
	ldi r17, 0x0f
	and r16, r17
show_pot:
	breq end_conv
	write_lcd_data 255
	dec r17
	dec r16
	rjmp show_pot
end_conv:
	tst r17
clean:
	breq start_conv
	write_lcd_data ' '
	dec r17
	rjmp clean

fim:
    inc r16
    rjmp fim


init_lcd:
	write_lcd_cmd LCD_POWER_UP
	delay_6ms
	write_lcd_cmd LCD_POWER_UP
	write_lcd_cmd LCD_POWER_UP
	write_lcd_cmd LCD_8BIT_2LINES
	write_lcd_cmd LCD_DOFF_COFF_BOFF
	write_lcd_cmd LCD_CLEAR
	rcall delay_2ms
	write_lcd_cmd LCD_MODE_INC
	write_lcd_cmd LCD_DON_CON_BON
	ret


_write_lcd_cmd:   ; cmd on R24
	out LCD_DATA, R24 
    cbi LCD_CTRL, LCD_RS  ; RS = 0
	sbi LCD_CTRL, LCD_EN  ; EN = 1
	rcall delay_1us
	cbi LCD_CTRL, LCD_EN ; EN = 0
	ret

_write_lcd_data:   ; cmd on R24
	out LCD_DATA, R24 
    sbi LCD_CTRL, LCD_RS  ; RS = 1
	sbi LCD_CTRL, LCD_EN  ; EN = 1
	rcall delay_1us
	cbi LCD_CTRL, LCD_EN ; EN = 0
	ret

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

mess1:
.db "Hello! ",0x00
mess2:
.db "Uergs...", 0x00,0x00

ADC_COMP:
	lds r19, ADCH
	ldi r16, (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (1 << ADIE) | (7 << ADPS0)
	sts ADCSRA, r16
	reti
	