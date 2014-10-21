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
#define OLED_write_address	0x78	// (OLED_address<<1)
#define OLED_read_address	0x79	// (OLED_address<<1)
#define OLED_cmd_registry	0x80	// for commands ?
#define OLED_val_registry	0x40	// for values ?

// add function names
volatile void OLED_send_cmd(char);
volatile void OLED_send_byte(char);
volatile void OLED_send_char();
volatile void OLED_send_string();
volatile void OLED_send_num(int32_t);
volatile void OLED_set_position(uint8_t, uint8_t);
volatile void OLED_set_pos_mid();
volatile void OLED_set_pos_0();
volatile void OLED_init();
volatile void OLED_clear();

#endif /* INCFILE1_H_ */ 