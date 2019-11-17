/*
 * display7.inc
 *
 *  Created: 09/09/2019 13:52:36
 *   Author: CLIENTE
 */ 

  .cseg

 .macro rolw
	clc
	rol @1
	rol @0
.endm

 .macro wait_button_press
	sbis @0, @1
	rjmp pc-1
.endm

.macro wait_button_release
	sbic @0, @1
	rjmp pc-1
.endm

.macro wait_button_click
	sbic @0, @1
	rjmp pc-1
	sbis @0, @1
	rjmp pc-1
.endm

segments_value_table: ; CA CB CC CD CE CF CG DP
.db 0x03, 0x9f, 0x25, 0x0d, 0x99, 0x49, 0x41, 0x1f, 0x01, 0x19, 0x11, 0xc1, 0x63, 0x85, 0x61, 0x71

.equ PORT_CA = PORTD
.equ BIT_CA  = 5
.equ PORT_CB = PORTB
.equ BIT_CB  = 3
.equ PORT_CC = PORTB
.equ BIT_CC  = 2
.equ PORT_CD = PORTB
.equ BIT_CD  = 1
.equ PORT_CE = PORTD
.equ BIT_CE  = 3
.equ PORT_CF = PORTD
.equ BIT_CF  = 6
.equ PORT_CG = PORTD
.equ BIT_CG  = 4
.equ PORT_DP = PORTD
.equ BIT_DP  = 7

init_display:
	sbi (PORT_CA-1), BIT_CA
	sbi (PORT_CB-1), BIT_CB
	sbi (PORT_CC-1), BIT_CC
	sbi (PORT_CD-1), BIT_CD
	sbi (PORT_CE-1), BIT_CE
	sbi (PORT_CF-1), BIT_CF
	sbi (PORT_CG-1), BIT_CG
	sbi (PORT_DP-1), BIT_DP
	ret

display_value:
	ldi zl, low(segments_value_table)
	ldi zh, high(segments_value_table)
	rolw zh, zl
	add zl, r24
	adc zh, r25
	lpm r18, z
	ldi r19, 0xff
	eor r18, r19
	; segmento A
	cbi PORT_CA, BIT_CA
	rol r18
	brcc pc+2
	sbi PORT_CA, BIT_CA
	; segmento B
	cbi PORT_CB, BIT_CB
	rol r18
	brcc pc+2
	sbi PORT_CB, BIT_CB
	;segmento C
	cbi PORT_CC, BIT_CC
	rol r18
	brcc pc+2
	sbi PORT_CC, BIT_CC
	;segmento D
	cbi PORT_CD, BIT_CD
	rol r18
	brcc pc+2
	sbi PORT_CD, BIT_CD
	; segmento E
	cbi PORT_CE, BIT_CE
	rol r18
	brcc pc+2
	sbi PORT_CE, BIT_CE
	; segmento F
	cbi PORT_CF, BIT_CF
	rol r18
	brcc pc+2
	sbi PORT_CF, BIT_CF
	;segmento C
	cbi PORT_CG, BIT_CG
	rol r18
	brcc pc+2
	sbi PORT_CG, BIT_CG
	;segmento PONTO
	cbi PORT_DP, BIT_DP
	rol r18
	brcc pc+2
	sbi PORT_DP, BIT_DP
	nop
	ret
