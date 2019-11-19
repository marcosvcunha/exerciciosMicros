/*
 * misc.inc
 *
 *  Created: 10/30/2018 11:41:34 AM
 *   Author: jlfragoso
 */ 

 #ifndef __MISC_INC__
 #define __MISC_INC__

 .macro nibble_to_hex
	andi @0, 0x0f
	ori @0, 0x30
	cpi @0, 0x3a
	brmi pc+2
	add @0, @1
	nop
.endm

.macro wait_button_press
	sbis @0, @1
	rjmp pc-1
.endm

.macro wait_button_release
	sbic @0, @1
	rjmp pc-1
.endm

.macro wait_button_press_and_release
	sbis @0, @1
	rjmp pc-1
	sbic @0, @1
	rjmp pc-1
.endm

.macro delay_us
	ldi r24, @0
	rcall _delay_us
.endm

.macro delay_6ms
	rcall delay_2ms
	rcall delay_2ms
	rcall delay_2ms
.endm

.cseg
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

buffer_flush:
	ldi r16, low(_START_BUFFER_ADDR)
	ldi r17, high(_START_BUFFER_ADDR)
	sts _buffer_head, r16
	sts (_buffer_head+1), r17
	sts _buffer_tail, r16
	sts (_buffer_tail+1), r17
	ldi r16, 0
	sts _buffer_flag, r16
	ret

buffer_write:  ; recebe r18 byte 
	ldi r16, 0xff
	lds r17, _buffer_flag
	cp r16, r17
	breq _buffer_write_ret
	inc r17
	sts _buffer_flag, r17
	lds yl, _buffer_head
	lds yh, (_buffer_head+1)
	st y+, r18 ; salvou no buffer
	ldi r16, low(_END_BUFFER_ADDR)
	ldi r17, high(_END_BUFFER_ADDR)
	cp r17, yh
	brne _buffer_write_end
	cp r16, yl
	brne _buffer_write_end
	ldi yl, low(_START_BUFFER_ADDR)
	ldi yh, high(_START_BUFFER_ADDR)
_buffer_write_end:
	sts _buffer_head, yl
	sts (_buffer_head+1), yh
_buffer_write_ret:
	ret

buffer_read: ; retorna byte r18
	lds r16, _buffer_flag
	tst r16
	breq _buffer_read_ret
	dec r16
	sts _buffer_flag, r16
	lds yl, (_buffer_tail)
	lds yh, (_buffer_tail+1)
	ld r18, y+ ; le caracter
	ldi r16, low(_END_BUFFER_ADDR)
	ldi r17, high(_END_BUFFER_ADDR)
	cp r17, yh
	brne _buffer_read_end
	cp r16, yl
	brne _buffer_read_end
	ldi yl, low(_START_BUFFER_ADDR)
	ldi yh, high(_START_BUFFER_ADDR)
_buffer_read_end:
	sts _buffer_tail, yl
	sts (_buffer_tail+1), yh
_buffer_read_ret:
	ret


int16_to_asc:
	ldi r19, 5 ; 5 casas decimais
_start_conv:
	ldi r18, 13  ; 12 bits
	ldi r16, 0x00
	ldi r17, 0xA0
	ldi r20, 0
	ldi r21, 0
_div10:
	cp r24, r16
	cpc r25, r17
	brlo _next_step
	sec
	adc r20, r1
	sub r24, r16
	sbc r25, r17
_next_step:
	clc
	ror r17
	ror r16
	clc
	rol r20
	rol r21
	dec r18
	brne _div10
	clc
	ror r21
	ror r20
	ori r24, 0x30
	push r24
	mov r24, r20
	mov r25, r21
	dec r19
	brne _start_conv
	pop r16
	st x+, r16
	pop r16
	st x+, r16
	pop r16
	st x+, r16
	pop r16
	st x+, r16
	pop r16
	st x+, r16
	ldi r16, 0
	st x+, r16
	ret

.dseg
.equ BUFFER_SIZE = 256
_START_BUFFER_ADDR:
_rbuffer: .byte BUFFER_SIZE
_END_BUFFER_ADDR:
_buffer_head: .byte 2
_buffer_tail: .byte 2
_buffer_flag: .byte 1
_asc_buffer : .byte 10
 #endif 
