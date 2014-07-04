/*
 * Accelerometer.h
 *
 * Created: 17.06.2013 11:03:25
 *  Author: SNFU
 */ 


#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

// #################################
// Accelerometer address
#define LSM303_ACC_ADDRESS_READ		0x33	// 0011 0011
#define LSM303_ACC_ADDRESS_WRITE	0x32	// 0011 0010

//Accelerometer registers
#define LSM303_ACC_CTRL_REG1		0x20
#define LSM303_ACC_CTRL_REG2		0x21
#define LSM303_ACC_CTRL_REG3		0x22
#define LSM303_ACC_CTRL_REG4		0x23
#define LSM303_ACC_CTRL_REG5		0x24
#define LSM303_ACC_CTRL_REG6		0x25
#define LSM303_ACC_REF				0x26
#define LSM303_ACC_STAT_REG			0x27
#define LSM303_ACC_OUT_X_L			0x28
#define LSM303_ACC_OUT_X_H			0x29
#define LSM303_ACC_OUT_Y_L			0x2A
#define LSM303_ACC_OUT_Y_H			0x2B
#define LSM303_ACC_OUT_Z_L			0x2C
#define LSM303_ACC_OUT_Z_H			0x2D


// Function prototypes
void write_acc_registry(char registry, char data);
void acc_start(void);
struct acc_readings_obj acc_reading(void);


#endif /* ACCELEROMETER_H_ */