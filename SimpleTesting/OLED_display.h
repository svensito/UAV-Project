/*
 * OLED_display.h
 *
 * Created: 06.10.2014 13:40:46
 *  Author: SNFU
 */ 


#ifndef OLED_DISPLAY_H_
#define OLED_DISPLAY_H_

// add defines
#define OLED_address		0x3C	// OLED address as per MiniWii data...
#define OLED_cmd_registry	0x80	// for commands ?
#define OLED_val_registry	0x40	// for values ?

// add function names
volatile void OLED_write_char(char char_in);
volatile void OLED_write_string(char string);
volatile void OLED_write_val_to_reg(uint8_t val, uint8_t reg);
volatile void OLED_init();

#endif /* INCFILE1_H_ */