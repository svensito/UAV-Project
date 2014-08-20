/*
 * Magnetometer.c
 *
 * Created: 17.06.2013 11:34:01
 *  Author: SNFU
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "GeneralSettings.h"
#include "uart.h"
#include "Magnetometer.h"
#include <math.h>

//################################
// variables
// mag reading
int8_t	mag_x_h		= 0;
uint8_t	mag_x_l		= 0;
int16_t mag_x		= 0;
int16_t	mag_x_deg	= 0;
int16_t mag_x_max	= +540;		// used for calibration
int16_t mag_x_min	= -640;		// used for calibration

int8_t	mag_y_h		= 0;
uint8_t	mag_y_l		= 0;
int16_t mag_y		= 0;
int16_t	mag_y_deg	= 0;
int16_t mag_y_max	= +620;		// used for calibration
int16_t mag_y_min	= -640;		// used for calibration

int8_t	mag_z_h		= 0;
uint8_t	mag_z_l		= 0;
int16_t mag_z		= 0;
int16_t	mag_z_deg	= 0;
int16_t mag_z_max	= +500;		// used for calibration
int16_t mag_z_min	= -500;		// used for calibration

int16_t mag_heading	= 0;
int16_t alpha_int = 0;
//##################################
// functions

// general write into magnetometer registry (fixed address)
void write_mag_registry(char registry, char data)
{
		i2c_start();
		i2c_send_address_write(LSM303_MAG_ADDRESS_WRITE);
		i2c_send_registry(registry);
		i2c_send_data(data);
		i2c_stop();
}

// start magnetometer -> setting registry to have modes properly
void mag_start(void)
{
		// Calibration of the Magnetometer
		
		
	
		// write_mag_registry(LSM303_MAG_CRA_REG,);	//		// CRA_REG (output data rate) is set to 100 default -> 15 Hz
		// write_mag_registry(LSM303_MAG_CRB_REG,);	//		// CRB_REG (gain setting)	is set to default 001	-> 
		write_mag_registry(LSM303_MAG_MR_REG, 0x00);		// MR_REG	(Mode Select)	is set to default 00	-> continuous conversion
															// To set the MR_REG is essential, so make sure it is set once...
															
															
		write_string_ln("mag started");
}

// read data from magnetometer
int16_t mag_read(void)
{
	i2c_start();
	i2c_send_address_write(LSM303_MAG_ADDRESS_WRITE);
	i2c_send_registry(LSM303_MAG_OUT_X_H);					// the magnetometer supports auto increment without any further info
	i2c_repeated_start();
	i2c_send_address_read(LSM303_MAG_ADDRESS_READ);
	mag_x_h = i2c_read_data_mak();
	mag_x_l = i2c_read_data_mak();
	mag_z_h = i2c_read_data_mak();
	mag_z_l = i2c_read_data_mak();
	mag_y_h = i2c_read_data_mak();
	mag_y_l = i2c_read_data_nmak();
	i2c_stop();
	
	mag_x = ((mag_x_h<<8)|(mag_x_l));
	mag_y = ((mag_y_h<<8)|(mag_y_l));
	mag_z = ((mag_z_h<<8)|(mag_z_l));
	
	// Calculating the heading with the calibrated data
	
	// Calculating the heading with atan2 function by math.h
	mag_heading = atan2(mag_y,mag_x)*(180/PI);
	if(mag_heading < 0) mag_heading += 360;
	
	return mag_heading;
	/*
	Do we need to add Z-axis for mag reading?
	*/ 
	
	/*	
	if((mag_x >= 0) && (mag_y >= 0))
	{
		heading = round((atan((float)mag_y/(float)mag_x))*(180/PI));
		heading = 360 - heading;
	}
	else if ((mag_x >= 0) && (mag_y < 0))
	{
		heading = -1*(round((atan((float)mag_y/(float)mag_x))*(180/PI)));
	}
	else if ((mag_x < 0) && (mag_y >= 0))
	{
		heading = round((atan((float)mag_y/(float)mag_x))*(180/PI));
		heading = 180 - heading;
		
	}
	else // if both are negative
	{
		heading = round((atan((float)mag_y/(float)mag_x))*(180/PI));
		heading = 180 - heading;
	}
	*/
	
	//write_var_ln(heading);
}