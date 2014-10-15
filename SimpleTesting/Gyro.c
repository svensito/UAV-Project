/*
 * Gyro.c
 *
 * Created: 16.06.2013 15:17:11
 *  Author: Sven
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "GeneralSettings.h"
#include "uart.h"
#include "Gyro.h"

// Gyro variables
// gyro reading
/*
volatile struct three_values
{
	int16_t p,q,r;
};
*/

int8_t	gyro_x_h	= 0;
uint8_t gyro_x_l	= 0;
int16_t gyro_x		= 0;
int16_t	gyro_p		= 0;

int8_t	gyro_y_h	= 0;
uint8_t gyro_y_l	= 0;
int16_t gyro_y		= 0;
int16_t	gyro_q		= 0;

int8_t	gyro_z_h	= 0;
uint8_t gyro_z_l	= 0;
int16_t gyro_z		= 0;
int16_t	gyro_r		= 0;	

// Gyro temperature
//int8_t	temperature	= 0;
//uint8_t temperature_0	= 50;

// Gyro Calibration Variables
// idea: count 100 samples (IMU needs to be steady) and calculate the offset
uint8_t cal_count = 0;
int16_t gyro_p_offset = 0;
int16_t gyro_q_offset = 0;
int16_t gyro_r_offset = 0;
uint8_t cal_complete = FALSE;	// Flag to cover if the calibration is complete

// Gyro functions

// reading registry general function
void read_gyro_registry(char registry)
{
	int data;
	i2c_start();
	i2c_send_address_write(L3GD20_ADDRESS_WRITE);
	i2c_send_registry(registry);
	i2c_repeated_start();
	i2c_send_address_read(L3GD20_ADDRESS_READ);
	data = i2c_read_data_nmak();
	i2c_stop();
	write_var_ln(data);
}

// writing registry general function
void write_gyro_registry(char registry, char data)
{
	i2c_start();
	i2c_send_address_write(L3GD20_ADDRESS_WRITE);
	i2c_send_registry(registry);
	i2c_send_data(data);
	i2c_stop();
	//write_string_ln("data wrote");
}

// Gyro intialization (start) routine
void gyro_start(void)
{
	// CTRL REG 1
	write_gyro_registry(L3GD20_CTRL_REG1, 0x0F);		// Enabling all axes read out and Power On
	// CTRL REG 2
	write_gyro_registry(L3GD20_CTRL_REG2, 0x20);		// Normal Mode of High Pass Filter (High Bits), Filter Rate (Low Bits 0-9), Output Data Rate to 100 Hz (default)
	// CTRL REG 5
	write_gyro_registry(L3GD20_CTRL_REG5, 0x10);		// High Pass Filter Enable (High Bit 1)
	write_string_ln("gyro started");
}
// Gyro temperature reading
// void temp_read(void)
// {
// 	i2c_start();
// 	i2c_send_address_write(L3GD20_ADDRESS_WRITE);
// 	i2c_send_registry(L3GD20_OUT_TEMP);
// 	i2c_repeated_start();
// 	i2c_send_address_read(L3GD20_ADDRESS_READ);
// 	temperature = i2c_read_data_nmak();
// 	i2c_stop();
// 	temperature = (temperature);
// 	write_var_ln(temperature);
// }

// Gyro turn rate reading
struct three_elements_obj gyro_read(void)
{
	i2c_start();
	i2c_send_address_write(L3GD20_ADDRESS_WRITE);
	i2c_send_registry(L3GD20_OUT_X_L + 0b10000000);		// add MSB to activate auto increment
	i2c_repeated_start();
	i2c_send_address_read(L3GD20_ADDRESS_READ);
	gyro_x_l = i2c_read_data_mak();
	gyro_x_h = i2c_read_data_mak();
	gyro_y_l = i2c_read_data_mak();
	gyro_y_h = i2c_read_data_mak();
	gyro_z_l = i2c_read_data_mak();
	gyro_z_h = i2c_read_data_nmak();					// add the end we give no master ack to finish communication
	i2c_stop();
	
	gyro_x = (gyro_x_h<<8) | (gyro_x_l);
	gyro_y = (gyro_y_h<<8) | (gyro_y_l);
	gyro_z = (gyro_z_h<<8) | (gyro_z_l);
	
	struct three_elements_obj turn_rates;
	turn_rates.p = gyro_x * 0.00875;						// the conversion comes from the default sensitivity value: FS = 250dps	-> 8.75mdps/digit
	turn_rates.q = gyro_y * 0.00875;
	turn_rates.r = gyro_z * 0.00875;						// see gyro datasheet page 10
	
	if(cal_complete == TRUE) 
	{
		turn_rates.p -= gyro_p_offset;
		turn_rates.q -= gyro_q_offset;
		turn_rates.r -= gyro_r_offset;
	}		
																											
	/*write_string("gyro_x reg:");write_var(gyro_x);write_string("  p: ");*///write_var(gyro_p);write_string(" deg/s \r\n ");
	return(turn_rates);
}

// Gyro calibration routine
void gyro_calibration(void)
{
	// defining the temporary stored turn rates
	struct three_elements_obj turn_rates_temp;
	while(cal_count < 10)
	{
		cal_count ++;
		turn_rates_temp = gyro_read();
		gyro_p_offset += turn_rates_temp.p;
		gyro_q_offset += turn_rates_temp.q;
		gyro_r_offset += turn_rates_temp.r;
	}
	gyro_p_offset /= cal_count;
	gyro_q_offset /= cal_count;
	gyro_r_offset /= cal_count;
	write_string_ln("Gyro Offset: ");
	write_string("p: ");write_var_ln(gyro_p_offset);
	write_string("q: ");write_var_ln(gyro_q_offset);
	write_string("r: ");write_var_ln(gyro_r_offset);
	_delay_ms(500);
	cal_complete = TRUE;
	write_string_ln("Gyro Cal Complete");
}