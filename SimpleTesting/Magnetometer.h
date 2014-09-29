/*
 * Magnetometer.h
 *
 * Created: 17.06.2013 11:33:52
 *  Author: SNFU
 */ 


#ifndef MAGNETOMETER_H_
#define MAGNETOMETER_H_

// #################################
// Magnetometer address
#define LSM303_MAG_ADDRESS_READ		0x3D	// 0011 1101	as per Pololu website and Datasheet
#define LSM303_MAG_ADDRESS_WRITE	0x3C	// 0011 1100

// Magnetometer Registers
#define LSM303_MAG_WHO_AM_I			0x0F
#define LSM303_MAG_CRA_REG			0x00
#define LSM303_MAG_CRB_REG			0x01
#define LSM303_MAG_MR_REG			0x02
#define LSM303_MAG_OUT_X_H			0x03
#define LSM303_MAG_OUT_X_L			0x04
#define LSM303_MAG_OUT_Z_H			0x05
#define LSM303_MAG_OUT_Z_L			0x06
#define LSM303_MAG_OUT_Y_H			0x07
#define LSM303_MAG_OUT_Y_L			0x08
#define LSM303_MAG_TEMP_H			0x31
#define LSM303_MAG_TEMP_L			0x32

// Function prototypes
void write_mag_registry(char registry, char data);
void mag_start(void);
volatile int16_t mag_read(float Phi_In, float Theta_In);

#endif /* MAGNETOMETER_H_ */