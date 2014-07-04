#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "GeneralSettings.h"
#include "uart.h"




// ###############################################
// UART COMMUNICATION

// general definition setting
uart_ready_flag = FALSE;
char GPS_info[][];
uint8_t GPS_Params = 10;

// UART initialization
void init_uart()
{
	// USART Initialization
	// USCSRB Register
	cli();	//that is needed essentially (turn off interrupts shortly)
	UCSRB = (0<<UCSZ2)|(1<<TXEN)|(1<<RXEN)|(1<<RXCIE);//|(1<<UDRIE);
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	// Need to be in line with the baudrate!
	// Baudrate: 2400 12 MHZ prozessor
	UBRRH = 0x00;
	UBRRL = 0x4E;
	
	sei();	//that is needed essentially (turn on interrupts globally)
	// das UDR auslesen, falls daten drin sind und verwerfen
	do
    {
        UDR;
    }
    while (UCSRA & (1 << RXC));	//solange das receive bit gesetzt ist

}
// UART receive interrupt routine
ISR (USART_RXC_vect)
{
	// the UDR is read in number	
    uart_reading_ascii = UDR;
	// the ascii number will be converted to char
	uart_reading = (char) uart_reading_ascii;
	
	if ((uart_reading != '\r') || (uart_reading != ','))	/*as we are using the Carriage Return, the terminal program needs to end a transmission with CR to be detected */
	{
		uart_ready_flag = FALSE;
		// write the reading into the string array on the position of the counter
		uart_word[uart_counter] = uart_reading;
		GPS_info[row][uart_counter] = uart_reading;
		// increase the counter (position of the string)
		uart_counter++;
	}
	else if (uart_reading == ',')
	{
		GPS_info[row][uart_counter] = '\0';
		row++;
		if(row == GPS_Params) row = 0;
	}
	else
	{
		/*
		// append a \0 to terminate the string
		uart_word[uart_counter] = '\0';
		// reset the counter
		uart_counter = 0;
		// show the main program that a string is available
		uart_ready_flag = TRUE;
		// check the reading
		//write_string(uart_word);
		*/
	}

}

// UART write char
extern void write_char(char *c)
{	
    while (!(UCSRA & (1<<UDRE)))  /* wait to send */
    {
    }
    
    UDR = c;                      /* Send char */
    	
}

// UART write string
extern void	write_string(char *s)
{
	while(*s)
	{
		write_char(*s);
		s++;
	}
	
}

// UART write string with carriage return and line feed
extern void write_string_ln(char *s)
{
	while(*s)
	{
		write_char(*s);
		s++;
	}
	write_string("\r\n");
}

// UART write signed Integer 16bit	//-> 16 bits will leave us with numbers up to 65536 !! and what happens with higher numbers?
extern void write_var(int16_t num)
{
	char k[15];			// define two byte
	itoa(num, k, 10);	// convert integer to asci -> decimal system
	write_string(k);	// send the byte
}

extern void write_var_ln(int32_t num)
{
		char k[31];			// define four bytes
		ltoa(num, k, 10);	// convert integer to asci -> decimal system
		write_string(k);	// send the byte
		write_string("\r\n");
}

extern void write_ubyte_ln(uint8_t num)
{
		char k[8];			// define two byte
		itoa(num, k, 10);	// convert integer to asci -> decimal system
		write_string(k);	// send the byte
		write_string("\r\n");
}

extern void write_ubyte(uint8_t num)
{
		char k[8];			// define two byte
		itoa(num, k, 10);	// convert integer to asci -> decimal system
		write_string(k);	// send the byte
}

extern void send_sbyte(int8_t num)
{
		    while (!(UCSRA & (1<<UDRE)))  /* wait to send */
		    {
		    }
		    
		    UDR = num;                      /* Send char */
}

extern void send_ubyte(uint8_t num)
{
	while (!(UCSRA & (1<<UDRE)))  /* wait to send */
	{
	}
			    
	UDR = num;                      /* Send byte */
}

extern void send_sshort(int16_t num)
{
	int8_t MSB;
	uint8_t LSB;
	LSB = num & (0xFF);
	MSB = num>>8;
	send_ubyte(LSB);send_sbyte(MSB);	
}

extern void send_ushort(uint16_t num)
{
	uint8_t MSB,LSB;
	LSB = num & (0xFF);
	MSB = num>>8;
	send_ubyte(LSB);send_ubyte(MSB);
}

extern void send_slong(int32_t num)
{
	// signed most significant bytes
	int8_t MSB;
	// unsigned central and least significant bytes
	uint8_t CSB2,CSB1,LSB;
	LSB = num & (0xFF);
	CSB1 = num & (0xFF00);
	CSB2 = num & (0xFF0000);
	MSB = num>>24;
	send_ubyte(LSB);send_ubyte(CSB1);send_ubyte(CSB2);send_sbyte(MSB);
}

extern void send_ulong(uint32_t num)
{
		uint8_t MSB,CSB2,CSB1,LSB;
		LSB = num & (0xFF);
		CSB1 = num & (0xFF00);
		CSB2 = num & (0xFF0000);
		MSB = num>>24;
		send_ubyte(LSB);send_ubyte(CSB1);send_ubyte(CSB2);send_ubyte(MSB);
}

// END UART
// ###########################################################