/*
 * port.h
 *
 * Created: 16.04.2013 13:11:25
 *  Author: snfu
 */ 


#ifndef PORT_H_
#define PORT_H_

void i2c_transmit(char address, char reg, char data);
unsigned char i2cRead(char address, char reg);
void i2c_initialize();
void i2c_start();
void i2c_repeated_start();
void i2c_repeated_start_OLED();
void i2c_send_address_read(char);
void i2c_write_val_to_reg_OLED(char,char,char);
void i2c_wait_transmission();
void i2c_send_address_write(char);
void i2c_send_registry(char);
void i2c_send_data(char);
void i2c_stop();
int i2c_read_data_nmak();
int i2c_read_data_mak();

#endif /* PORT_H_ */