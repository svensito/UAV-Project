/*
 * SimpleTesting.c
 *
 * Created: 10.07.2014 19:07:30
 *  Author: snfu
 
  This project is for the GPS receiving ATMEGA.
  The data from the GPS will be read, analyzed and provided per I2C interface.
 */ 



//------------------------------------------------------
// DEFINE SECTION
//#define F_CPU	12000000UL



//------------------------------------------------------

//------------------------------------------------------
// INCLUDE SECTION
#include "GeneralSettings.h"
#include <avr/io.h>
#include <avr/delay.h>
#include "uart.h"
#include "port.h"
#include <avr/interrupt.h>
#include <string.h>
#include <avr/wdt.h>
//------------------------------------------------------

//------------------------------------------------------
// Local declarations and variables


//------------------------------------------------------


//------------------------------------------------------



//-----------------------------------
// Watchdog
// Is used to secure in case there is a disturbance and the controller does not react.
// Enabling the Watchdog
void WDT_on(void)
{
	// Set the Watchdog Enable Bit
	// Seting the Prescaler Bits: 101 will give around 0.5 seconds timeout
		WDTCR |= (1<<WDE)|(1<<WDP2)|(0<<WDP1)|(1<<WDP0);
}

// Disabling the Watchdog
void WDT_off(void)
{
	/* command to Toggle the Watchdog */
	wdt_reset();
	
	/* Write logical one to WDTOE and WDE */
	WDTCR |= (1<<WDTOE) | (1<<WDE);
	/* Turn off WDT */
	WDTCR = 0x00;
	
}
//-----------------------------------


int main(void)
{	
	
	// Initializing
	init_uart();
	
	// Checking the Reset Status
	if (MCUCSR & (1<<WDRF)) write_string("WdogRst");
	else if(MCUCSR & (1<<PORF)) write_string("PwrUpRst");
	else if(MCUCSR & (1<<BORF)) write_string("BrwnOutRst");
	else if(MCUCSR & (1<<EXTRF)) write_string("ExtRst");
	else write_var(MCUCSR);
	// Resetting the Status Register
	MCUCSR &= ~((1<<WDRF)|(1<<PORF)|(1<<BORF)|(1<<EXTRF));
	_delay_ms(500);
	
	
	i2c_initialize_as_slave();
	
	// Turn On the Watchdog
	WDT_on();
	
	while(1)
    {
		// Program Code (infinite loop)
			
			write_string_ln(GPS_string[GPS_STATUS]);
			
			// Toggling the Watchdog (Reset)
			wdt_reset();
    }
}