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
void OLED_send_cmd(char);
void OLED_send_byte(char);
//void OLED_send_char();
void OLED_send_char_A();
void OLED_send_char_B();
void OLED_send_char_C();
void OLED_send_char_D();
void OLED_send_char_E();
void OLED_send_char_F();
void OLED_send_char_G();
void OLED_send_char_H();
void OLED_send_char_I();
void OLED_send_char_J();
void OLED_send_char_K();
void OLED_send_char_L();
void OLED_send_char_M();
void OLED_send_char_N();
void OLED_send_char_O();
void OLED_send_char_P();
void OLED_send_char_Q();
void OLED_send_char_R();
void OLED_send_char_S();
void OLED_send_char_T();
void OLED_send_char_U();
void OLED_send_char_V();
void OLED_send_char_W();
void OLED_send_char_X();
void OLED_send_char_Y();
void OLED_send_char_Z();
void OLED_send_char_SP();
void OLED_send_num_0();
void OLED_send_num_1();
void OLED_send_num_2();
void OLED_send_num_3();
void OLED_send_num_4();
void OLED_send_num_5();
void OLED_send_num_6();
void OLED_send_num_7();
void OLED_send_num_8();
void OLED_send_num_9();
//extern void OLED_send_string();
extern void OLED_send_num(int32_t);
extern void OLED_set_position(uint8_t, uint8_t);
extern void OLED_set_pos_mid();
extern void OLED_set_pos_0();
void OLED_page(uint8_t);
void OLED_page_BAT();
extern void OLED_init();
extern void OLED_clear();

#endif /* OLED_DISPLAY_H_ */ 