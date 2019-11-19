/**
* @file lcd_i2c.h
* Header file for using LCD with RGB Backlight over I2C interface.
* @author Joao Leonardo Fragoso
* @date 2018-06-21
* @depends i2c_master.h
*/

#ifndef __LCD_I2C__
#define __LCD_I2C__
#include <avr/io.h>
#include <util/delay.h>
#include "i2c_master.h"
/**
* A LCD I2C controller class.
* A complete class for accessing and controlling
* an I2C LCD with RGB back light. Allow multiple LCDs
* on I2C with distinct address. Use constructor to 
* define LCD address and to initialize.
* Used device address from internet 
* https://forum.dexterindustries.com/t/solved-grove-lcd-rgb-backlight-unknown-i2c-addresses/2879/3
* 0x3e - LCD Address
* 0x62 - RGB Slave address
* 0x70 - LCD all call address (it’s on at startup and cannot be addressed individually)
* 0x03 - Software reset address (it’s on at startup and cannot be addressed individually)
* @depend i2c_master 
*/
class lcd_i2c
{

public:
	static const uint8_t ADDRESS		= (0x7c>>1); /**< Default LCD address, if no address is provided */
	static const uint8_t REG_CMD		= (0x80);   /**< register address for sending LCD commands */
	static const uint8_t REG_DATA		= (0x40);  /**< register address for sending LCD data */
	static const uint8_t RGB_ADDRESS	= (0xc4>>1); /**< Default RGB back light address */
	static const uint8_t RGB_MODE1		= (0x00);  /**< RGB mode 1 register address */
	static const uint8_t RGB_MODE2		= (0x01); /**< RGB mode 2 register address */
	static const uint8_t RGB_OUTPUT		= (0x08); /**< RGB output register address */
	static const uint8_t RGB_RED		= (0x04); /**< RGB red color intensity register address */
	static const uint8_t RGB_GREEN		= (0x03); /**< RGB green color intensity register address */
	static const uint8_t RGB_BLUE		= (0x02); /**< RGB blue color intensity register address */
	// command codes
	static const uint8_t POWER_UP		= (0x28); /**< LCD power up command code */
	static const uint8_t BIT8_2LINES	= (0x38); /**< LCD 8 bits interface and 2 lines command code */
	static const uint8_t BIT8_1LINE		= (0x30); /**< LCD 8 bits interface and 1 line command code */
	static const uint8_t BIT4_2LINES	= (0x28); /**< LCD 4 bits interface and 2 line command code */
	static const uint8_t BIT4_1LINE		= (0x20); /**< LCD 4 bits interface and 1 line command code */
	static const uint8_t DON_COFF_BOFF	= (0x0C); /**< LCD command for display on, cursor off and blink off */
	static const uint8_t DON_COFF_BON	= (0x0D); /**< LCD command for display on, cursor off and blink on */
	static const uint8_t DON_CON_BOFF	= (0x0E); /**< LCD command for display on, cursor on and blink off */
	static const uint8_t DON_CON_BON	= (0x0F); /**< LCD command for display on, cursor on and blink on */
	static const uint8_t DOFF_COFF_BOFF	= (0x08); /**< LCD command for display off, cursor off and blink off */
	static const uint8_t DOFF_COFF_BON	= (0x09); /**< LCD command for display off, cursor off and blink on */
	static const uint8_t DOFF_CON_BOFF	= (0x0A); /**< LCD command for display off, cursor on and blink off */
	static const uint8_t DOFF_CON_BON	= (0x0B); /**< LCD command for display off, cursor on and blink on */
	static const uint8_t MODE_INC		= (0x06); /**< LCD command for auto increment mode */
	static const uint8_t MODE_INC_SHIFT	= (0x07); /**< LCD command for auto increment and shift left mode */
	static const uint8_t MODE_DEC		= (0x04); /**< LCD command for auto decrement mode */
	static const uint8_t MODE_DEC_SHIF	= (0x05); /**< LCD command for auto decrement and shift right mode */
	static const uint8_t CLEAR			= (0x01); /**< LCD clear command */
	static const uint8_t RETURN_HOME	= (0x02); /**< LCD return cursor home command */
	static const uint8_t SET_CURSOR		= (0x80); /**< LCD set cursor position command code */
	static const uint8_t LINE_0			= (0x00); /**< LCD line 0 position value */
	static const uint8_t LINE_1			= (0x40); /**< LCD line 1 position value */
protected:
	uint8_t _lcd_address; /**< LCD effective address of an object */
	uint8_t _rgb_address; /**< RGB effective address of an object */
public:
	/**
	* LCD controller constructor.
	* This constructor will initialize LCD on I2C bus.
	* It's take around 7ms to start and return.
	* @param lcd_address the address for accessing LCD on I2C
	* @param rgb_address the address for accessing RGB backlight controller
	*/
	lcd_i2c(uint8_t lcd_address=ADDRESS, uint8_t rgb_address = RGB_ADDRESS);
	/**
	* Set back light color. 
	* The color is defined by RGB value
	* @param red red intensity
	* @param green green intensity
	* @param blue blue intensity
	*/
	void setColor(uint8_t red, uint8_t green, uint8_t blue);
	/**
	* Set cursor position. 
	* Set a new position for the cursor on LCD.
	* @param row the new row index (starts on 0)
	* @param column the new column index (from 0 to 15)
	*/
	void setCursor(uint8_t row, uint8_t column);
	/**
	* Return cursor to home position.
	*/
	void returnHome();
	/**
	* Send a command to LCD.
	* This method is used to send any command to
	* LCD over the TWI/I2C interface.
	* @param command the command (byte) to be sent
	*/
	void cmd(uint8_t command);
	/**
	* Send a data to LCD.
	* This method is used to send any data to 
	* LCD. Data normally is the characters to be
	* shown on LCD.
	* @param data a byte to be sent
	*/
	void write(uint8_t data);
	/**
	* Send multiple data to LCD.
	* This method is used to send data in 
	* burst mode to LCD. It is used to print 
	* a string on LCD.
	* The vector should be null-ended.
	* @param msg a pointer to a null-ended char vector
	*/
	void write(const char *msg);
	/**
	* Clear the LCD.
	*/
	void clear();
	/**
	* Default destructor.
	* @todo clear and turn-off LCD on destructor
	*/
	~lcd_i2c();
protected:
	/**
	* Initialize the LCD.
	* This method provides the correct start-up
	* for the LCD. This also calls init_rgb()
	* to start back light.
	*/
	void init();
	/**
	* Initialize RGB Backlight.
	* Sets up the back light to be controlled
	* by PWMs and defines the start up color.
	*/
	void init_rgb();
}; //lcd_i2c

#endif //__LCD_I2C__
