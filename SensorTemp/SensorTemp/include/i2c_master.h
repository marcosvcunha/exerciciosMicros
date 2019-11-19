/**
* @file i2c_master.h
* Header file for using ATMEGA I2C (or TWI) hardware support
* as master for a I2C bus.
* I2C Master Controller
* @author Joao Leonardo Fragoso
* @date 2018-06-11
*/

#ifndef __I2C_MASTER__
#define __I2C_MASTER__


#ifndef  F_CPU
/**
* Clock frequency.
* A macro for defining clock frequency. Users 
* must correct set this macro in theirs projects,
* since this used for compute register values to
* achieve protocol frequency.
*/
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <util/twi.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

/**
* @addtogroup _comm
* @{
*/
/**
* TWI/I2C frequency. This define frequency
* for communicating over TWI.
*/
#define F_SCL 100000UL // SCL frequency
/**
* TWI Prescaler. This macro defines
* the prescaler that will be used during
* implementation.
*/
#define TWI_Prescaler 1
/**
* TWBR Value macro. This macros computes
* the register value for initializing TWB register
* for correct communication speed.
* @todo correct formula for 2560. It will not work for Prescaler != 1
*/
#define TWBR_Value ((((F_CPU / F_SCL) / (4^TWI_Prescaler)) - 16 ) / 2)
// Old define for ATMEGA328P...
//#define TWBR_Value ((((F_CPU / F_SCL) / (4^TWI_Prescaler)) - 16 ) / 2)


#define __NOP() do { __asm__ __volatile__ ("nop"); } while (0)
extern "C" void WDT_vect(void) __attribute__((signal));

/**
* I2C/TWI master control interface.
* A complete interface for controlling TWI protocol
* using ATMEGA as a master. 
* No object is need. Just call the desire methods and
* ensure to call init at beginning of your code.
*/
class i2c_master {
	protected:
		static volatile bool abortI2C;
		/**
		* Start a TWI transaction.
		* This method generates start conditions and wait for
		* TWI Interruption bit to be set.
		*/
		static void start(void);
		
		/**
		* Writes an 8-bit data on TWI bus.
		* Method is used to write all (address, reg value and data).
		*/
		static void write(uint8_t data);
		
		/**
		* Read an 8-bit data from TWI bus without acknowledging.
		* Method is used to catch a byte from bus without acknowledging
		* and allowing multiple reads (burst) in the same transaction.
		*/
		static uint8_t read_nack(void);
		
		/**
		* Read an 8-bit data from TWI bus and acknowledge data.
		* Method is used to catch a byte from bus with acknowledging.
		* Allows a simple read or ending multiple reads (burst) transaction.
		*/
		static uint8_t read_ack(void);
		
		/**
		* Stops TWI transaction.
		* This method is used to generate correct
		* stop condition on TWI protocol.
		*/
		static void stop(void);

		static inline void wait_end_transmission(void)  {
			wdt_enable(1);
			loop_until_bit_is_set(TWCR, TWINT);
			do {
				__NOP();
				__NOP();
			} while(bit_is_clear(TWCR,TWINT) && !abortI2C);
			wdt_disable();
		}
	public:
	
		/**
		* Initialize TWI support.
		* Set SCL as output port and initialize all
		* TWI registers according defined macros.
		*/
		static void init(void);
				
		/**
		* Send a single data to a device.
		* @param address the device address.
		* @param data byte to be sent.
		*/
		static void send(uint8_t address, uint8_t data);
		
		/**
		* Sends multiples bytes to a device.
		* Multiple bytes will be sent to a device in burst mode.
		* @param address a byte with device address
		* @param data a pointer to a byte vector with data to be sent
		* @param length number of bytes to be sent.
		*/
		static void send(uint8_t address, const uint8_t* data, uint16_t length);
		
		/**
		* Receive a single byte from a device.
		* @param address the device address
		* @param data a byte reference to store received byte
		*/
		static void receive(uint8_t address, uint8_t &data);
		
		/**
		* Receives multiples bytes from a device.
		* Multiple bytes will be read from a device in burst mode.
		* @param address a byte with device address
		* @param data a pointer to a byte vector with data to be sent
		* @param length number of bytes to be sent
		*/
		static void receive(uint8_t address, uint8_t* data, uint16_t length);
		
		/**
		* Write a single register on a device.
		* @param address the device address
		* @param regaddr the register address to be written
		* @param data    the value to be written on the register
		*/
		static void writeReg(uint8_t address, uint8_t regaddr, uint8_t data);

		/**
		* Write multiples registers on a device.
		* From start register address multiple bytes will be sent to a
		* device in burst mode.
		* @param address the device address
		* @param regaddr the starting register address
		* @param data    the starting address of a byte vector
		* @param length  number of bytes to be sent
		*/
		static void writeReg(uint8_t address, uint8_t regaddr, const uint8_t* data, uint16_t length);
		
		/**
		* Read a single register from a device.
		* @param address the device address
		* @param regaddr the register address to be read
		* @param data a byte reference to store received byte
		*/
		static void readReg(uint8_t address, uint8_t regaddr, uint8_t &data);
		
		/**
		* Write multiples registers on a device.
		* From start register address multiple bytes will be sent to a
		* device in burst mode. Data is a vector of 16-bit (word) data.
		* Each data will be split in two writes and the eight most-significant bits
		* are written first.
		* @param address the device address
		* @param regaddr the starting register address
		* @param data    the starting address of a words vector
		* @param length  number of words into the vector
		*/		
		static void writeReg(uint8_t address, uint8_t regaddr, const uint16_t* data, uint16_t length);
		
		/**
		* Read multiples registers from a device.
		* Multiple register from a devices in a burst mode. The number of bytes to be
		* read is defined by the parameter length.
		* @param address the device address
		* @param regaddr the starting register address
		* @param data    the starting address of a vector to store the received data
		* @param length  number of bytes to be read
		*/
		static void readReg(uint8_t address, uint8_t regaddr, uint8_t* data, uint16_t length);
		
		/**
		* Read a register to obtain a single bit.
		* In fact, a single register is read and after the desired bit is masked, so only
		* a bit will be stored into data. Bit is kept on its position.
		* @param address the device address
		* @param regaddr the register address to be read
		* @param bit_num the bit index to be kept
		* @param data a byte reference to store result
		*/
		static void readBit(uint8_t address, uint8_t regaddr, uint8_t bit_num, uint8_t &data);
		
		/**
		* Read a register to obtain some bits.
		* In fact, a single register is read and the desired bits are masked out, so 
		* only selected bits will be stored into data.
		* The bit start value should be the MSB to be read and the length indicates number
		* of bits from the the MSB. After masked, remains bits are shifted right and aligned
		* to the first bit (LSB) of a byte. Result is stored into data.
		* @code
		* Example: bit_start = 4 and length=3
		* BITS  7 6 5 4 3 2 1 0
		* Read  X X X X X X X X
		* Mask       X X X
		* Shift  0 0 0 0 0 X X X  <-- result
		* @endcode
		*
		* @param address the device address
		* @param regaddr the register address to be read
		* @param bit_start the bit index start to be kept
		* @param length    the number of bits to be kept from start bit
		* @param data a byte reference to store result
		*/
		static void readBits(uint8_t address, uint8_t regaddr, uint8_t bit_start, uint8_t length, uint8_t &data);

		/**
		* Write a single bit into a device register.
		* The current device register is read. the correspondent bit (bit_num) is set if data is not zero,
		* otherwise the bit is cleared. So, the resulted value is written back to device register.
		* @param address the device address
		* @param regaddr the register address to be written
		* @param bit_num the bit index
		* @param data value to bit. If data is zero, bit is cleared, otherwise bit is set.
		*/
		static void writeBit(uint8_t address, uint8_t regaddr, uint8_t bit_num, uint8_t data);
		
		/**
		* Write selected bits into a device register.
		* The current device register is read. The selected bits are masked off register value and these bits
		* are kept into data value. So, both values are or-ed togheter.
		* The result is then written back to register.
		*
		* @code
		* Example: bit_start = 4 and length=3
		* BITS        7 6 5 4 3 2 1 0
		* data        D D D D D D D D
		* data_masked 0 0 0 D D D 0 0 
		* Read        X X X X X X X X
		* Masked      X X X 0 0 0 X X
		* or-ed       X X X D D D X X  <-- result to be written
		* @endcode
		*
		* @param address the device address
		* @param regaddr the register address to be updated
		* @param bit_start the bit index start
		* @param length    the number of bits 
		* @param data bit values to be written
		*/
		static void writeBits(uint8_t address, uint8_t regaddr, uint8_t bit_start, uint8_t length, uint8_t data);
		
		friend void ::WDT_vect(void);

};

/**
* @}
*/

#endif // __I2C_MASTER___
