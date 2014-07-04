/*
 * Barometer.c
 *
 * Created: 17.06.2013 11:36:31
 *  Author: SNFU
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "GeneralSettings.h"
#include "uart.h"
#include "Barometer.h"

//############################
// variables
int8_t	baro_p_h	= 0;	// MSB
uint8_t	baro_p_c	= 0;	// CSB
uint8_t	baro_p_l	= 0;	// LSB
float	baro_p_l_converted	= 0;	// needs to be a float as LSB is a 4 bit fractional value

uint8_t baro_cal_flag = FALSE;
uint8_t baro_cal_count = 0;
int16_t baro_offset = 0;

int16_t baro_p		= 0;	// cast as a unsigned short
int8_t	source_int	= 0;	// Data ready register value
int16_t baro_previous = 0;

//############################
// functions

// writing 'data' to 'registry' by setting Baro Address Write
void write_baro_registry(char registry, char data)
{
			i2c_start();
			i2c_send_address_write(MPL3115A2_ADDRESS_WRITE);
			i2c_send_registry(registry);
			i2c_send_data(data);
			i2c_stop();	
}

// setting the mode of the barometer
void baro_start()
{
			write_baro_registry(MPL3115A2_CTRL_REG1, 0b10111001);			// data:	SBYB (Bit0) = 1 -> Normal Active Mode	
																			//			ALT (Bit7)	= 1	-> Altimeter Mode (gives output in m)
																			// -> Oversampling active (as per Sparkfun sample code)
			write_baro_registry(MPL3115A2_CTRL_REG4, 0b10000000);			// enables data ready interrupt flag
			write_baro_registry(MPL3115A2_BAR_IN_MSB, 0xC6);				// calibrates the sensor according to the sea level pressure
			write_baro_registry(MPL3115A2_BAR_IN_LSB, 0x5B);				// calibrates the sensor according to the sea level pressure
			write_baro_registry(MPL3115A2_PT_DATA_CFG, 0x07);
			write_string_ln("baro started");
	
}

int16_t baro_read()
{
			uint8_t status = 0;
			// first we need to see if data is available!
			i2c_start();
			i2c_send_address_write(MPL3115A2_ADDRESS_WRITE);
			i2c_send_registry(MPL3115A2_STATUS);
			i2c_repeated_start();
			i2c_send_address_read(MPL3115A2_ADDRESS_READ);
			status = i2c_read_data_nmak();
			i2c_stop();
			//write_string("status: ");write_var_ln(status);
			// if bit 2 of the Status Registry is set to 1, then new data is available
			// -> mask for the bit, then check the value of the mask (here -> 2nd bit, gives 2
			while(baro_cal_flag == FALSE)
			{
				if(status == 238)
				{
					i2c_start();
					i2c_send_address_write(MPL3115A2_ADDRESS_WRITE);
					i2c_send_registry(MPL3115A2_OUT_P_MSB);					// the barometer supports auto increment without any further info
					i2c_repeated_start();
					i2c_send_address_read(MPL3115A2_ADDRESS_READ);
					baro_p_h = i2c_read_data_mak();							// the output is called p, anyways as we set the ALT Bit, we will read an Altitude in m
					baro_p_c = i2c_read_data_mak();
					baro_p_l = i2c_read_data_nmak();
					i2c_stop();
					// Determine the Offset
					baro_p = (baro_p_h << 8) |(baro_p_c);
					baro_cal_flag = TRUE;
					return baro_p;
				}
			}
			if(status == 238)
			{
				//write_string_ln("read ");
				i2c_start();
				i2c_send_address_write(MPL3115A2_ADDRESS_WRITE);
				i2c_send_registry(MPL3115A2_OUT_P_MSB);					// the barometer supports auto increment without any further info
				i2c_repeated_start();
				i2c_send_address_read(MPL3115A2_ADDRESS_READ);
				baro_p_h = i2c_read_data_mak();							// the output is called p, anyways as we set the ALT Bit, we will read an Altitude in m
				baro_p_c = i2c_read_data_mak();
				baro_p_l = i2c_read_data_nmak();
				i2c_stop();
						
				// as per datasheet MPL3115A2 page 5, 1 bit in Alt mode equals 0.0625 m
						
				//baro_p_l_converted = (int16_t)()/16;
						
				// the lowest byte is only using the highest 4 bits
				// further these 4 bits are the fractional value
				// as we do not want the fractional part, we do not read the
				// p_l bit.
				baro_p = (baro_p_h << 8) |(baro_p_c);
				baro_previous = baro_p-baro_offset;
				//write_var_ln(baro_previous);
				return (baro_previous);
				// This value can be used as absolute altitude above sea level standard pressure -> acquisition will be done every second, no need for permanent polling
			}
			else return baro_previous; 		
			
}

void baro_calibration()
{
	baro_offset = baro_read();
	write_string("baro_offset ");write_var_ln(baro_offset);
	//baro_cal_flag = TRUE;
}