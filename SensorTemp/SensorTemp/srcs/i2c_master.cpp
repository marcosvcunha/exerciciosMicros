/**
* @file i2c_master.cpp
* I2C Master Controller class implementation.
* @author Joao Leonardo Fragoso
* @date 2018-06-21
*/

#include "i2c_master.h"

volatile bool i2c_master::abortI2C = false;

void i2c_master::init(void)
{
	// define port scl/sda as output
	DDRC |= 0x30;
	
	// write 1 to output (idle)
	PORTC |= 0x30; 
	// define port scl/sda as input
	//DDRD &= 0xfc;
	// write 1 to output
	//PORTD |= 0x03;
	// set frequency of TWI
	TWBR = (uint8_t) 72; //TWBR_Value;
	TWSR = 0x00;
	TWCR = _BV(TWEN) | _BV(TWIE) | _BV(TWEA);
}


void i2c_master::start()
{
	abortI2C = false;
	// reset TWI control register
	TWCR = 0;
	// transmit START condition 
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	// wait for end of transmission
	wait_end_transmission();
}

void i2c_master::write(uint8_t data)
{
	if (!abortI2C) {
	// load data into data register
	TWDR = data;
	// start transmission of data
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	loop_until_bit_is_set(TWCR, TWINT);
	
//	if( (TWSR & 0xF8) != TW_MT_DATA_ACK ){ return false; }
	
	//return true;
	}
}

uint8_t i2c_master::read_ack(void)
{
	if (!abortI2C) {
 	// start TWI module and acknowledge data after reception
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA); 
	// wait for end of transmission
	wait_end_transmission();
	}
	// return received data from TWDR
	return TWDR;
}

uint8_t i2c_master::read_nack(void)
{	
	if (!abortI2C) {
	// start receiving without acknowledging reception
	TWCR = _BV(TWINT) | _BV(TWEN);
	// wait for end of transmission
	wait_end_transmission();
	}
	// return received data from TWDR
	return TWDR;
}

void i2c_master::send(uint8_t address, const uint8_t* data, uint16_t length)
{
	// start transaction
	start();
	// send device address with WRITE bit
	write(address<<1 | TW_WRITE);
	// send data bytes in burst
	for (uint16_t i = 0; i < length; i++)
		write(data[i]);
	// stop transaction
	stop();
}

void i2c_master::send(uint8_t address, uint8_t data)
{
	// start transaction
	start();
	// send device address with write bit
	write(address<<1 | TW_WRITE);
	// send byte
	write(data);
	// stop transaction
	stop();
}

void i2c_master::receive(uint8_t address, uint8_t* data, uint16_t length)
{
	start();
	write(address<<1 | TW_READ);
	for (uint16_t i = 0; i < (length-1); i++)
			data[i] = read_ack();
	data[(length-1)] = read_nack();
	stop();
}

void i2c_master::receive(uint8_t address, uint8_t &data)
{
	start();
	write(address<<1 | TW_READ);
	data = read_nack();
	stop();
}

void i2c_master::writeReg(uint8_t address, uint8_t regaddr, const uint8_t* data, uint16_t length)
{
	start();
	write(address<<1 | TW_WRITE);
	write(regaddr);
	for (uint16_t i = 0; i < length; i++)
		write(data[i]);
	stop();
}

void i2c_master::writeReg(uint8_t address, uint8_t regaddr, uint8_t data)
{

	start();
	write(address<<1 | TW_WRITE);
	write(regaddr);
	write(data);
	stop();
}


void i2c_master::readReg(uint8_t address, uint8_t regaddr, uint8_t* data, uint16_t length)
{
	start();
	write(address<<1 | TW_WRITE);
	write(regaddr);
	start();
	write(address<<1 | TW_READ);
	for (uint16_t i = 0; i < (length-1); i++)
		data[i] = read_ack();
	data[(length-1)] = read_nack();
	stop();
}

void i2c_master::readReg(uint8_t address, uint8_t regaddr, uint8_t &data)
{
	start();
	write(address<<1 | TW_WRITE);
	write(regaddr);
	start();
	write(address<<1 | TW_READ);
	data = read_nack();
	stop();
}

void i2c_master::readBit(uint8_t address, uint8_t regaddr, uint8_t bit_num, uint8_t &data)
{
	readReg(address, regaddr, data);
	data &= _BV(bit_num); // mask other bits
}

void i2c_master::readBits(uint8_t address, uint8_t regaddr, uint8_t bit_start, uint8_t length, uint8_t &data)
{   
 // 01101001 read byte
// 76543210 bit numbers
//    xxx   args: bitStart=4, length=3
//    010   masked
//   -> 010 shifted
	if (length > 0) {
		uint8_t mask=((1<<length)-1) << (bit_start - length + 1);
		readReg(address, regaddr, data);
		data &= mask;
		data >>= (bit_start - length + 1);
	}
}

void i2c_master::writeBit(uint8_t address, uint8_t regaddr, uint8_t bit_num, uint8_t data)
{
	uint8_t b;
	readReg(address, regaddr, b);
	b = (data) ? (b | _BV(bit_num)) : (b & ~_BV(bit_num));
	writeReg(address,regaddr, b);
}

void i2c_master::writeBits(uint8_t address, uint8_t regaddr, uint8_t bit_start, uint8_t length, uint8_t data)
{
	    //      010 value to write
	    // 76543210 bit numbers
	    //    xxx   args: bitStart=4, length=3
	    // 00011100 mask byte
	    // 10101111 original value (sample)
	    // 10100011 original & ~mask
	    // 10101011 masked | value
		if (length>0) {
			uint8_t b;
			uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
			data <<= (bit_start - length + 1);
			data &= mask;
			readReg(address, regaddr, b);
			b &= ~mask;
			b |= data;
			writeReg(address, regaddr, b);
		}
}

void i2c_master::writeReg(uint8_t address, uint8_t regaddr, const uint16_t* data, uint16_t length)
{
	if (length>0)	 {
		start();
		write(address<<1 | TW_WRITE);
		write(regaddr);
		length <<= 1; // multiply by 2
		for (uint16_t i = 0; i < length; i++) {
			write((uint8_t)(data[i]>>8));
			i++;
			write((uint8_t)data[i]);
		}
		stop();
	}
}

void i2c_master::stop(void)
{
	// transmit STOP condition
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

ISR(WDT_vect)
{
}


