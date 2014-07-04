/*
 * Barometer.h
 *
 * Created: 17.06.2013 11:36:45
 *  Author: SNFU
 */ 


#ifndef BAROMETER_H_
#define BAROMETER_H_


// #################################
// Barometer Addresses
#define MPL3115A2_ADDRESS_READ		0xC1	// 1100 0001
#define MPL3115A2_ADDRESS_WRITE		0xC0	// 1100 0000

// Barometer Registers
#define MPL3115A2_STATUS			0x00
#define MPL3115A2_OUT_P_MSB			0x01
#define MPL3115A2_OUT_P_CSB			0x02
#define MPL3115A2_OUT_P_LSB			0x03
#define MPL3115A2_OUT_T_MSB			0x04
#define MPL3115A2_OUT_T_LSB			0x05
#define MPL3115A2_STATUS_DR			0x06
#define MPL3115A2_OUT_P_DELTA_MSB	0x07
#define MPL3115A2_OUT_P_DELTA_CSB	0x08
#define MPL3115A2_OUT_P_DELTA_LSB	0x09
#define MPL3115A2_OUT_T_DELTA_MSB	0x0A
#define MPL3115A2_OUT_T_DELTA_LSB	0x0B
#define MPL3115A2_WHO_AM_I			0x0C
#define MPL3115A2_INT_SOURCE		0x12
#define MPL3115A2_PT_DATA_CFG		0x13
#define MPL3115A2_BAR_IN_MSB		0x14
#define MPL3115A2_BAR_IN_LSB		0x15
#define MPL3115A2_P_TGT_MSB			0x16
#define MPL3115A2_P_TGT_MSB			0x17
#define MPL3115A2_T_TGT				0x18
#define MPL3115A2_P_WND_MSB			0x19
#define MPL3115A2_P_WND_LSB			0x1A
#define MPL3115A2_T_WND				0x1B
#define MPL3115A2_CTRL_REG1			0x26
#define MPL3115A2_CTRL_REG2			0x27
#define MPL3115A2_CTRL_REG3			0x28
#define MPL3115A2_CTRL_REG4			0x29
#define MPL3115A2_CTRL_REG5			0x2A

// Function prototypes
void write_baro_registry(char registry, char data);
void baro_start(void);
volatile int16_t baro_read(void);
void baro_calibration(void);

#endif /* BAROMETER_H_ */