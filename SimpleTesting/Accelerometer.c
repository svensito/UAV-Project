/*
 * Accelerometer.c
 *
 * Created: 17.06.2013 11:03:12
 *  Author: SNFU
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "GeneralSettings.h"
#include "uart.h"
#include "Accelerometer.h"

// acc reading
int8_t	acc_x_h		= 0;
uint8_t	acc_x_l		= 0;
int16_t acc_x		= 0;
int16_t	acc_x_g		= 0;

int8_t	acc_y_h		= 0;
uint8_t	acc_y_l		= 0;
int16_t acc_y		= 0;
int16_t	acc_y_g		= 0;

int8_t	acc_z_h		= 0;
uint8_t	acc_z_l		= 0;
int16_t acc_z		= 0;
int16_t	acc_z_g		= 0;


/*
volatile struct acc_readings
{
	int16_t a_x,a_y,a_y;
};
*/
/* Accelerometer Reading*/
// Functions

void write_acc_registry(char registry, char data)
{
	i2c_start();
	i2c_send_address_write(LSM303_ACC_ADDRESS_WRITE);
	i2c_send_registry(registry);
	i2c_send_data(data);
	i2c_stop();
	//write_string_ln("data wrote");
}

void acc_start(void)
{
	write_acc_registry(LSM303_ACC_CTRL_REG1, 0b01000111);	// enabling 50Hz output rate, normal power mode, all axes enabled
	write_acc_registry(LSM303_ACC_CTRL_REG4, 0b00010000);	// setting Scale to +-4G, LSM303 Datasheet Page 26
	write_string_ln("acc started");
}

struct acc_readings_obj acc_reading(void)
{
	i2c_start();
	i2c_send_address_write(LSM303_ACC_ADDRESS_WRITE);
	i2c_send_registry(LSM303_ACC_OUT_X_L+0b10000000);		// add MSB to activate auto increment
	i2c_repeated_start();
	i2c_send_address_read(LSM303_ACC_ADDRESS_READ);
	acc_x_l = i2c_read_data_mak();
	acc_x_h = i2c_read_data_mak();
	acc_y_l = i2c_read_data_mak();
	acc_y_h = i2c_read_data_mak();
	acc_z_l = i2c_read_data_mak();
	acc_z_h = i2c_read_data_nmak();					// add the end we give no master ack to finish communication
	i2c_stop();
	
	acc_x = ((acc_x_h<<8) | (acc_x_l))>>4;			// to combine high and low bits and to shift over not used bits
	acc_y = ((acc_y_h<<8) | (acc_y_l))>>4;
	acc_z = ((acc_z_h<<8) | (acc_z_l))>>4;
	
	acc_x_g = acc_x * 2; // in milli g => 1/1000 g		// As our FS is set to 01 (+-4G), the resolution is 2mg/digit,  LSM303 Datasheet Page 9
	acc_y_g = acc_y * 2;
	acc_z_g = acc_z * 2;
	
	struct acc_readings_obj acc_values;
	acc_values.a_x = acc_x_g;
	acc_values.a_y = acc_y_g;
	acc_values.a_z = acc_z_g;
	
	return(acc_values);
}