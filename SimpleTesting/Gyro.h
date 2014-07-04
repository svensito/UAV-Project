/*
 * Gyro.h
 *
 * Created: 16.06.2013 15:15:41
 *  Author: Sven
 */ 


#ifndef GYRO_H_
#define GYRO_H_

// #################################
// Gyro addresses
//#define L3G4200D_ADDRESS_SA0_LOW  (0xD0 >> 1)
//#define L3G4200D_ADDRESS_SA0_HIGH (0xD2 >> 1)
#define L3GD20_ADDRESS_READ		0xD7		// 1101 0011	as per Pololu website and Datasheet of L3GD20
#define L3GD20_ADDRESS_WRITE	0xD6		//

// Gyro registers
#define L3GD20_WHO_AM_I			0x0F
#define L3GD20_CTRL_REG1		0x20
#define L3GD20_CTRL_REG2		0x21
#define L3GD20_CTRL_REG3		0x22
#define L3GD20_CTRL_REG4		0x23
#define L3GD20_CTRL_REG5		0x24
#define L3GD20_OUT_TEMP			0x26
#define L3GD20_STATUS_REG		0x27
#define L3GD20_OUT_X_L			0x28
#define L3GD20_OUT_X_H			0x29
#define L3GD20_OUT_Y_L			0x2A
#define L3GD20_OUT_Y_H			0x2B
#define L3GD20_OUT_Z_L			0x2C
#define L3GD20_OUT_Z_H			0x2D

//###################################
// Gyro Functions
/* Gyro reading*/

void gyro_start(void);
void temp_read(void);
void read_gyro_registry(char registry);
void write_gyro_registry(char registry, char data);
struct three_elements_obj gyro_read(void);
void gyro_calibration(void);

#endif /* GYRO_H_ */