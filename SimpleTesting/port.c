#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>

#include "GeneralSettings.h"

void i2c_initialize()
{
	// Set the Baudrate Register
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
	
	// Enable the TWI Module
	TWCR |= (1<<TWEA)|(1<<TWIE)|(1<<TWEN);	// as per Arduino code!
	
	// Arduino additionally enables internal Pull Ups? Is this needed? Arduino is the only source where I found this.
}

void i2c_initialize_as_slave()
{
	// Set the Baudrate Register
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
	
	// Write the TWAR Register with TWI identifier -> Check that no other unit uses this identifier
	TWAR = ATM32_SLAVE_ADDR1<<1;	// Shift by one as the LSB is not dedicated for the Address
	
	// Enable the TWI Module
	TWCR |= (1<<TWEA)|(1<<TWIE)|(1<<TWEN);	// as per Arduino code!
	
	// Arduino additionally enables internal Pull Ups? Is this needed? Arduino is the only source where I found this.
}

void i2c_start()
{
	TWCR = 0xA4;                                                // send a start bit on i2c bus
	// TWCR |= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);					// identical to 0xA4
	while(!(TWCR & 0x80));                // wait for confirmation of transmit
	// while(!(TWCR & (1<<TWINT)));								// identical to above
	while((TWSR & 0xF8) != 0x08);								// This comes out of ATMEGA Datasheet to check for the Status Register
	
}

void i2c_repeated_start()
{
	TWCR = 0xA4;                                                // send a start bit on i2c bus
	// TWCR |= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);					// identical to 0xA4
	while(!(TWCR & 0x80)) ;                // wait for confirmation of transmit
	
}

void i2c_send_address_read(char address)
{
	TWDR = address;												// load address of i2c device into data register
	//TWCR = 0x84;												// transmit
	TWCR |= (1<<TWINT) | (1<<TWEN) | (1<<TWEA);				// as per Datasheet from ATMEGA32
	while(!(TWCR & 0x80));				    // wait for confirmation of transmit
	// while(!(TWCR & (1<<TWINT)));								// identical to above
	while(((TWSR & 0xF8) != 0x40));
	// 0x40 = Read mode agreed	0x18 = write mode agreed
		
}

void i2c_send_address_write(char address)
{
	TWDR = address;												// load address of i2c device into data register
	TWCR = 0x84;												// transmit
	// identical to TWCR = (1<<TWINT) | (1<<TWEN);				// as per Datasheet from ATMEGA32
	while(!(TWCR & 0x80));				    // wait for confirmation of transmit
	// while(!(TWCR & (1<<TWINT)));								// identical to above
	while(((TWSR & 0xF8) != 0x18)) ;
	// 0x40 = Read mode agreed	0x18 = write mode agreed
}

void i2c_send_registry(char reg)
{
	TWDR = reg;													// load registry ID device into data register
	TWCR = 0x84;                                                // transmit
	while(!(TWCR & 0x80));                 // wait for confirmation of transmit
	while(((TWSR & 0xF8) != 0x28));
}

void i2c_send_data(char data)
{
	TWDR = data;												// load data into data register
	TWCR = 0x84;                                                // transmit
	while(!(TWCR & 0x80));                 // wait for confirmation of transmit
	while(((TWSR & 0xF8) != 0x28)) ;
}

int i2c_read_data_nmak()
{
	TWCR = (1<<TWINT)|(1<<TWEN);	//|(1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	//while(!(TWCR & 0x80))write_string_ln("H3");
	//while(((TWSR & 0xF8) != 0x50)) write_var_ln(TWSR);
	int data;
	data = TWDR;
	return data;
}

int i2c_read_data_mak()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while(!(TWCR & (1<<TWINT)));
	//while(!(TWCR & 0x80))write_string_ln("H3");
	//while(((TWSR & 0xF8) != 0x50)) write_var_ln(TWSR);
	int data;
	data = TWDR;
	return data;
}

void i2c_stop()
{
	TWCR |= (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);									// Try (as per Datasheet)
	//TWCR = 0x94;                                                  // stop bit
	while(TWCR & (1<<TWSTO));					// From Pete Fleury I2C master
}
