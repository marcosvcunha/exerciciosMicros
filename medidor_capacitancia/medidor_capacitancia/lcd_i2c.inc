/*
 * lcd_i2c.inc
 *
 *  Created: 10/30/2018 11:34:34 AM
 *   Author: jlfragoso
 */ 
 
 #ifndef __LCD_I2C_INC__
 #define __LCD_I2C_INC__

.equ LCD_ADDRESS		= (0x7c>>1)
.equ RGB_ADDRESS        = (0xc4>>1)
.equ LCD_POWER_UP		= 0x28
.equ LCD_8BIT_2LINES	= 0x38 
.equ LCD_8BIT_1LINE		= 0x30  ; power_up
.equ LCD_4BIT_2LINES	= 0x28 
.equ LCD_4BIT_1LINE		= 0x20 
.equ LCD_DON_COFF_BOFF	= 0x0C  ;--display on, cursor off, blink off
.equ LCD_DON_COFF_BON	= 0x0D  ;--display on, cursor off, blink on
.equ LCD_DON_CON_BOFF	= 0x0E  ;--display on, cursor on, blink off
.equ LCD_DON_CON_BON	= 0x0F  ;--display on, cursor on, blink on
.equ LCD_DOFF_COFF_BOFF	= 0x08  ;--display off, cursor off, blink off
.equ LCD_DOFF_COFF_BON	= 0x09  ;--display off, cursor off, blink on
.equ LCD_DOFF_CON_BOFF	= 0x0A  ;--display off, cursor on, blink off
.equ LCD_DOFF_CON_BON	= 0x0B  ;--display off, cursor on, blink on
.equ LCD_MODE_INC		= 0x06
.equ LCD_MODE_INC_SHIFT = 0x07
.equ LCD_MODE_DEC		= 0x04
.equ LCD_MODE_DEC_SHIF	= 0x05
.equ LCD_CMD_CLEAR		= 0x01
.equ LCD_RETURN_HOME	= 0x02
.equ LCD_CMD_SET_CURSOR = 0x80
.equ LCD_LINE_0         = 0x00
.equ LCD_LINE_1         = 0x40
.equ RGB_RED			= 0x04
.equ RGB_GREEN			= 0x03
.equ RGB_BLUE			= 0x02


 .include "twi_master.inc"
 .include "misc.inc"

 .macro write_lcd_cmd
	ldi r26, @0
	ldi r24, LCD_ADDRESS
	ldi r25, 0x80
	rcall twi_write_slave_reg
	delay_us 50
.endm

.macro write_lcd_cmd_reg
	mov r26, @0
	ldi r24, LCD_ADDRESS
	ldi r25, 0x80
	rcall twi_write_slave_reg
	delay_us 50
.endm

.macro write_lcd_data
	ldi r26, @0
	ldi r24, LCD_ADDRESS
	ldi r25, 0x40
	rcall twi_write_slave_reg
	delay_us 50
.endm

.macro write_lcd_data_reg
	mov r26, @0
	ldi r24, LCD_ADDRESS
	ldi r25, 0x40
	rcall twi_write_slave_reg
	delay_us 50
.endm

.macro write_rgb_reg
	ldi r25, @0
	ldi r26, @1
	ldi r24, RGB_ADDRESS
	rcall twi_write_slave_reg
.endm

.macro rgb_set_color
	write_rgb_reg RGB_RED, @0
	write_rgb_reg RGB_GREEN, @1
	write_rgb_reg RGB_BLUE, @2
.endm

.macro lcd_set_cursor
	ldi r24, @0
	ldi r25, @1
	rcall _lcd_set_cursor
.endm

.cseg
lcd_init:
	write_lcd_cmd LCD_POWER_UP
	delay_6ms
	write_lcd_cmd LCD_POWER_UP
	delay_us 100
	write_lcd_cmd LCD_POWER_UP
	write_lcd_cmd LCD_POWER_UP
	write_lcd_cmd LCD_8BIT_2LINES
	write_lcd_cmd LCD_DOFF_COFF_BOFF
	write_lcd_cmd LCD_CMD_CLEAR
	rcall delay_2ms
	write_lcd_cmd LCD_MODE_INC
	write_lcd_cmd LCD_DON_COFF_BOFF
	rcall rgb_init
	ret

rgb_init:
	write_rgb_reg 0x00,0x00
	write_rgb_reg 0x08,0xff
	write_rgb_reg 0x01,0x20
	rgb_set_color 0x00,0x00,0xff
	ret

lcd_clear:
	write_lcd_cmd LCD_CMD_CLEAR
	rcall delay_2ms
	ret

_lcd_set_cursor:   ;r24 linha r25 coluna
	ldi  r26, LCD_CMD_SET_CURSOR
	andi r25, 0x0f
	andi r24, 0x01
	breq _continue
	ori r26, LCD_LINE_1
_continue:
	or r26, r25
	write_lcd_cmd_reg r26
	ret

lcd_write_str_flash: ; endereco da str em Z
	; alinhar o end (x2)
	clc
	rol zl
	rol zh
_l1_str_from_flash:
	lpm r24, z+ ;le char
	tst r24
	brne _p1_str ; se nao for nulo imprime
	ret
_p1_str: ; imprime e volta
	write_lcd_data_reg r24
	rjmp _l1_str_from_flash

lcd_write_str_sram: ; endereco da str em Z
_l1_str_from_sram:
	ld r24, z+ ;le char
	tst r24
	brne _p2_str ; se nao for nulo imprime
	ret
_p2_str: ; imprime e volta
	write_lcd_data_reg r24
	rjmp _l1_str_from_sram

lcd_write_reg_hex:
	push r24 ; guarda valor original
	swap r24
	andi r24, 0x0f
	ori  r24, 0x30
	cpi r24, 0x3a
	brmi _c1 ; continue
	ldi r25, 0x07
	add r24, r25
_c1: 
	write_lcd_data_reg r24
	pop r24
	andi r24, 0x0f
	ori  r24, 0x30
	cpi r24, 0x3a
	brmi _c2 ; continue
	ldi r25, 0x07
	add r24, r25
_c2: 
	write_lcd_data_reg r24
	ret

 #endif

