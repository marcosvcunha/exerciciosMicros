/*
 * twi_master.inc
 *
 *  Created: 10/30/2018 10:45:48 AM
 *   Author: jlfragoso
 */ 

 #ifndef __TWI_MASTER_H__
 #define __TWI_MASTER_H__

 .macro _wait_twi
	lds r24, TWCR
	sbrs r24, TWINT
	rjmp pc-3
.endm

 .cseg

 TWI_ISR:
	reti

twi_init:
	sbi PORTC, PC4
	sbi PORTC, PC5
	ldi r24, 72 
	sts TWBR, r24
	ldi r24, 0
	sts TWSR, r24
	ldi r24, (1<< TWEN) | (1 << TWEA) | (1 << TWIE)
	sts TWCR, r24
	ret

twi_start:
	ldi r24, 0
	sts TWCR, r24
	ldi r24, (1<<TWINT) | (1<<TWSTA) | (1<<TWEN)
	sts TWCR, r24
	_wait_twi
	ret

twi_stop:
	ldi r24, (1<<TWINT) | (1<<TWSTO) | (1<<TWEN)
	sts TWCR, r24
	ret

twi_write_byte: ;; valor no R24
	sts TWDR, r24
	ldi r24, (1<<TWINT) | (1<<TWEN)
	sts TWCR, r24
	_wait_twi
	ret

twi_read_nack:
	ldi r24, (1<<TWINT) | (1<<TWEN)
	sts TWCR, r24
	_wait_twi
	lds r24, TWDR
	ret

twi_read_ack:
	ldi r24, (1<<TWINT) | (1<<TWEN) | (1<<TWEA)
	sts TWCR, r24
	_wait_twi
	lds r24, TWDR
	ret

twi_write_slave_reg: ;r24 slv add r25 reg add r26 data
	push r26
	push r25
	push r24
	call twi_start
	pop r24
	clc  ;; add last bit as 0 to write
	rol r24 ;;
	call twi_write_byte  ;; write slave addr
	pop r24
	call twi_write_byte ;; reg addr
	pop r24
	call twi_write_byte ;; data
	call twi_stop
	ret

twi_read_slave_reg: ;r24 slv add r25 reg return data r24
	push r24
	push r25
	push r24
	call twi_start
	pop r24
	clc  ;; add last bit as 0 to write
	rol r24 ;;
	call twi_write_byte  ;; write slave addr
	pop r24
	call twi_write_byte ;; reg addr
	call twi_start
	pop r24
	sec
	rol r24
	call twi_write_byte ;; write slave addr || 1
	call twi_read_nack
	push r24
	call twi_stop
	pop r24
	ret

#endif
