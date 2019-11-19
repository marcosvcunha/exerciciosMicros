/**
* @file lcd_i2c.cpp
* I2C LCD Controller class implementation
* @author Joao Leonardo Fragoso
* @date 2018-06-21
*/

#include "lcd_i2c.h"
#include <avr\interrupt.h>

// default constructor
lcd_i2c::lcd_i2c(uint8_t lcd_address, uint8_t rgb_address)
{
	// set lcd address
	_lcd_address = lcd_address;
	// set rgb address
	_rgb_address = rgb_address;
	// call init
	init();
} //lcd_i2c

// default destructor
lcd_i2c::~lcd_i2c()
{
} //~lcd_i2c

void lcd_i2c::init_rgb()
{
	////cli();
	// init back light
	i2c_master::writeReg(_rgb_address, RGB_MODE1, 0);
	// set leds controllable by both PWM and GRPPWM registers
	i2c_master::writeReg(_rgb_address, RGB_OUTPUT, 0xFF);
	// set MODE2 values ????
	// 0010 0000 (BMBLNK to 1 -- blink mode)
	i2c_master::writeReg(_rgb_address, RGB_MODE2, 0x20);
	////sei();
}

void lcd_i2c::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
	////cli();
	i2c_master::writeReg(_rgb_address, RGB_RED, red);
	i2c_master::writeReg(_rgb_address, RGB_GREEN, green);
	i2c_master::writeReg(_rgb_address, RGB_BLUE, blue);
	////sei();
}

void lcd_i2c::setCursor(uint8_t row, uint8_t column)
{
	if (column < 16 && row < 2) {
		column |= ((SET_CURSOR) | (row?LINE_1:LINE_0));
	}
	cmd(column);
}

void lcd_i2c::returnHome()
{
	cmd(RETURN_HOME);
}

void lcd_i2c::cmd(uint8_t command)
{
	////cli();
	i2c_master::writeReg(_lcd_address, REG_CMD, command);
	////sei();
	_delay_us(50);
}

void lcd_i2c::write(uint8_t data)
{
	////cli();
	i2c_master::writeReg(_lcd_address, REG_DATA, data);
	////sei();
	_delay_us(50);
}

void lcd_i2c::write(const char *msg)
{
	while((*msg)) {
		////sei();
		i2c_master::writeReg(_lcd_address, REG_DATA, (uint8_t) (*msg));
		////cli();
		msg++;
		_delay_us(50);
	}
}

void lcd_i2c::init() {
	cmd(POWER_UP);
	_delay_ms(6);
	cmd(POWER_UP);
	_delay_us(100);
	cmd(POWER_UP);
	//cmd(POWER_UP);
	cmd(BIT8_2LINES);
	cmd(DOFF_COFF_BOFF);
	cmd(CLEAR);
	_delay_ms(2);
	cmd(MODE_INC);
	cmd(DON_COFF_BOFF);
	init_rgb();
	setColor(0xff, 0xff, 0x00);
}

void lcd_i2c::clear()
{
	
	////cli();
	cmd(CLEAR);
	////sei();
	_delay_ms(2);
}

