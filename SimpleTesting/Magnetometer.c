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
int8_t	mag_x_h		= 0;		// high bits of mag x reading
uint8_t	mag_x_l		= 0;		// low bits of mag x reading
int16_t mag_x_meas	= 0;		// the value read on the x axis of the magnetometer
int16_t	mag_x_geo	= 0;		// the geodetic reference frame value of x
int16_t	max_x_geo_z	= 0;		// the geodetic reference frame value of x determined by z axis value

int8_t	mag_y_h		= 0;		// high bits of mag y reading
uint8_t	mag_y_l		= 0;		// low bits of mag y reading
int16_t mag_y_meas	= 0;		// the value read on the y axis of the magnetometer
int16_t	mag_y_geo	= 0;		// the geodetic reference frame value of y
int16_t max_y_geo_z	= 0;		// the geodetic reference frame value of y determined by z axis value

int8_t	mag_z_h		= 0;		// high bits of mag z reading
uint8_t	mag_z_l		= 0;		// low bits of mag z reading
int16_t mag_z_meas	= 0;		// the value read on the z axis of the magnetometer

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
int16_t mag_read(float Phi_In, float Theta_In)
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
	
	mag_x_meas = ((mag_x_h<<8)|(mag_x_l));
	mag_y_meas = ((mag_y_h<<8)|(mag_y_l));
	mag_z_meas = ((mag_z_h<<8)|(mag_z_l));
	
	// Calculating the heading
	
	// Heading needs to be compensated -> aircraft (measurement) frame is moving -> geodetic frame values needed
	mag_x_geo = (cos(Theta_In*(PI/180))*1000*mag_x_meas)/1000;//+(sin(Theta_In*(PI/180))*sin(Phi_In*(PI/180))*100*mag_y_meas)+(sin(Phi_In*(PI/180))*sin(Theta_In*(PI/180))*100*mag_z_meas);		// indefinite for 90 degree values!
	
	mag_y_geo = (cos(Phi_In*(PI/180))*1000*mag_y_meas)/1000;//-(sin(Phi_In*(PI/180))*100*mag_z_meas);		// indefinite for 90 degree value!
	
	max_x_geo_z = sin(Theta_In*(PI/180))*mag_x_meas;	// indefinite for 0 degree values!
	max_y_geo_z = sin(Phi_In*(PI/180))*mag_y_meas;		// indefinite for 0 degree value!
	
	// Calculating the heading with atan2 function by math.h
	mag_heading = -atan2(mag_y_geo,mag_x_geo)*(180/PI);
	//write_var(mag_heading);write_string(";");write_var(mag_x_geo);write_string(";");write_var(mag_y_geo);write_string_ln(";");
	// for a reading in between 0 - 360 deg:
	if (mag_heading < 0) 
	{
		mag_heading += 360;
	}
		
	return mag_heading;

}