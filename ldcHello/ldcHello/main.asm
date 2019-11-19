;
; LCD_Test.asm
;
; Created: 9/11/2017 9:15:52 AM
; Author : jlfragoso
;

.org 0x0000
	jmp start
.org 0x0016
	jmp timer_int
	jmp timer_int
	jmp timer_int
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

; Replace with your application code
.org INT_VECTORS_SIZE
start:
	eor r0,r0
	eor r1,r1
	eor r2,r2
	eor r3,r3
	eor r4,r4
	ldi r24,0xff
	out ddrd, r24
	ldi r24, 0x03
	out ddrb, r24
	rcall init_lcd
	delay_us 50
	write_lcd_data 'C'
	write_lcd_data 'r'
	write_lcd_data 'o'
	write_lcd_data 'n'
	write_lcd_data 'o'
	rcall _delay_us
	; iniciando time
	ldi r16, high(62499)
	sts OCR1AH, r16
	ldi r16, low(62499)
	sts OCR1AL, r16
	ldi r16, (1<<OCIE1A); |(1<<TOIE1)
	sts TIMSK1,r16
	sts TCCR1A, r1
	ldi r16, (1 << WGM12) | (4<<CS10)
	sts TCCR1B, r16
	;; comecou a contar
	sei
fim:
	write_lcd_cmd 0xC0
	mov r24, r4
	swap r24
	andi r24, 0x0f
	ori r24, 0x30
	rcall _write_lcd_data
	delay_us 50

	mov r24, r4
	andi r24, 0x0f
	ori r24, 0x30
	rcall _write_lcd_data
	delay_us 50

	write_lcd_data ':'

	mov r24, r3
	swap r24
	andi r24, 0x0f
	ori r24, 0x30
	rcall _write_lcd_data
	delay_us 50

	mov r24, r3
	andi r24, 0x0f
	ori r24, 0x30
	rcall _write_lcd_data
	delay_us 50

	delay_6ms
	delay_6ms
	delay_6ms
	delay_6ms
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
	write_lcd_cmd LCD_DON_COFF_BOFF
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

timer_int:
	ldi r16, 0x0f
	and r16, r3
	cpi r16, 0x09
	brne _incrsec
	ldi r16, 0xf0
	and r3, r16
	mov r16, r3
	cpi r16, 0x50
	brne _incrdsec
	mov r3, r1

	ldi r16, 0x0f
	and r16, r4
	cpi r16, 0x09
	brne _incrmin
	ldi r16, 0xf0
	and r4, r16
	mov r16, r4
	cpi r16, 0x50
	brne _incrdmin
	mov r4, r1
	rjmp _timer_end
_incrdmin:
	ldi r16, 0x10
	add r4, r16
	rjmp _timer_end
_incrmin:
	inc r4
	rjmp _timer_end
_incrdsec:
	ldi r16, 0x10
	add r3, r16
	rjmp _timer_end
_incrsec:
	inc r3
_timer_end:
	reti
mess1:
.db "Hello! ",0x00
mess2:
.db "Uergs...", 0x00,0x00
