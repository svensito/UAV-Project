/*
 * OLED_display.c
 *
 * Created: 06.10.2014 13:42:31
 *  Author: SNFU
 */ 

//------------------------------------------------------
// INCLUDE SECTION
#include "GeneralSettings.h"
#include <avr/io.h>
#include <avr/delay.h>
#include "uart.h"
#include "ServoControl.h"
#include "port.h"
#include <avr/interrupt.h>
#include <string.h>
#include "OLED_display.h"
//------------------------------------------------------

// buffer
static char buffer = 0;

// this look up is needed for 
// modified look up for needs -> source http://www.asciitable.com/index/asciifull.gif
const uint8_t Font_Lookup[][5] = { // Refer to "Times New Roman" Font Database... 5 x 7 font
	{ 0x00,0x00,0x00,0x00,0x00},	// As "Space" is the 32nd element within the ascii table, we are subtractin 32 in the formula
	{ 0x00,0x00,0x4F,0x00,0x00}, //   (  1)  ! - 0x0021 Exclamation Mark	
	{ 0x00,0x07,0x00,0x07,0x00}, //   (  2)  " - 0x0022 Quotation Mark
	{ 0x14,0x7F,0x14,0x7F,0x14}, //   (  3)  # - 0x0023 Number Sign
	{ 0x24,0x2A,0x7F,0x2A,0x12}, //   (  4)  $ - 0x0024 Dollar Sign
	{ 0x23,0x13,0x08,0x64,0x62}, //   (  5)  % - 0x0025 Percent Sign
	{ 0x36,0x49,0x55,0x22,0x50}, //   (  6)  & - 0x0026 Ampersand
	{ 0x00,0x05,0x03,0x00,0x00}, //   (  7)  ' - 0x0027 Apostrophe
	{ 0x00,0x1C,0x22,0x41,0x00}, //   (  8)  ( - 0x0028 Left Parenthesis
	{ 0x00,0x41,0x22,0x1C,0x00}, //   (  9)  ) - 0x0029 Right Parenthesis
	{ 0x14,0x08,0x3E,0x08,0x14}, //   ( 10)  * - 0x002A Asterisk
	{ 0x08,0x08,0x3E,0x08,0x08}, //   ( 11)  + - 0x002B Plus Sign
	{ 0x00,0x50,0x30,0x00,0x00}, //   ( 12)  , - 0x002C Comma
	{ 0x08,0x08,0x08,0x08,0x08}, //   ( 13)  - - 0x002D Hyphen-Minus
	{ 0x00,0x60,0x60,0x00,0x00}, //   ( 14)  . - 0x002E Full Stop
	{ 0x20,0x10,0x08,0x04,0x02}, //   ( 15)  / - 0x002F Solidus
	{ 0x3E,0x51,0x49,0x45,0x3E}, //   ( 16)  0 - 0x0030 Digit Zero
	{ 0x00,0x42,0x7F,0x40,0x00}, //   ( 17)  1 - 0x0031 Digit One
	{ 0x42,0x61,0x51,0x49,0x46}, //   ( 18)  2 - 0x0032 Digit Two
	{ 0x21,0x41,0x45,0x4B,0x31}, //   ( 19)  3 - 0x0033 Digit Three
	{ 0x18,0x14,0x12,0x7F,0x10}, //   ( 20)  4 - 0x0034 Digit Four
	{ 0x27,0x45,0x45,0x45,0x39}, //   ( 21)  5 - 0x0035 Digit Five
	{ 0x3C,0x4A,0x49,0x49,0x30}, //   ( 22)  6 - 0x0036 Digit Six
	{ 0x01,0x71,0x09,0x05,0x03}, //   ( 23)  7 - 0x0037 Digit Seven
	{ 0x36,0x49,0x49,0x49,0x36}, //   ( 24)  8 - 0x0038 Digit Eight
	{ 0x06,0x49,0x49,0x29,0x1E}, //   ( 25)  9 - 0x0039 Dight Nine
	{ 0x00,0x36,0x36,0x00,0x00}, //   ( 26)  : - 0x003A Colon
	{ 0x00,0x56,0x36,0x00,0x00}, //   ( 27)  ; - 0x003B Semicolon
	{ 0x08,0x14,0x22,0x41,0x00}, //   ( 28)  < - 0x003C Less-Than Sign
	{ 0x14,0x14,0x14,0x14,0x14}, //   ( 29)  = - 0x003D Equals Sign
	{ 0x00,0x41,0x22,0x14,0x08}, //   ( 30)  > - 0x003E Greater-Than Sign
	{ 0x02,0x01,0x51,0x09,0x06}, //   ( 31)  ? - 0x003F Question Mark
	{ 0x32,0x49,0x79,0x41,0x3E}, //   ( 32)  @ - 0x0040 Commercial At
	{ 0x7E,0x11,0x11,0x11,0x7E}, //   ( 33)  A - 0x0041 Latin Capital Letter A
	{ 0x7F,0x49,0x49,0x49,0x36}, //   ( 34)  B - 0x0042 Latin Capital Letter B
	{ 0x3E,0x41,0x41,0x41,0x22}, //   ( 35)  C - 0x0043 Latin Capital Letter C
	{ 0x7F,0x41,0x41,0x22,0x1C}, //   ( 36)  D - 0x0044 Latin Capital Letter D
	{ 0x7F,0x49,0x49,0x49,0x41}, //   ( 37)  E - 0x0045 Latin Capital Letter E
	{ 0x7F,0x09,0x09,0x09,0x01}, //   ( 38)  F - 0x0046 Latin Capital Letter F
	{ 0x3E,0x41,0x49,0x49,0x7A}, //   ( 39)  G - 0x0047 Latin Capital Letter G
	{ 0x7F,0x08,0x08,0x08,0x7F}, //   ( 40)  H - 0x0048 Latin Capital Letter H
	{ 0x00,0x41,0x7F,0x41,0x00}, //   ( 41)  I - 0x0049 Latin Capital Letter I
	{ 0x20,0x40,0x41,0x3F,0x01}, //   ( 42)  J - 0x004A Latin Capital Letter J
	{ 0x7F,0x08,0x14,0x22,0x41}, //   ( 43)  K - 0x004B Latin Capital Letter K
	{ 0x7F,0x40,0x40,0x40,0x40}, //   ( 44)  L - 0x004C Latin Capital Letter L
	{ 0x7F,0x02,0x0C,0x02,0x7F}, //   ( 45)  M - 0x004D Latin Capital Letter M
	{ 0x7F,0x04,0x08,0x10,0x7F}, //   ( 46)  N - 0x004E Latin Capital Letter N
	{ 0x3E,0x41,0x41,0x41,0x3E}, //   ( 47)  O - 0x004F Latin Capital Letter O
	{ 0x7F,0x09,0x09,0x09,0x06}, //   ( 48)  P - 0x0050 Latin Capital Letter P
	{ 0x3E,0x41,0x51,0x21,0x5E}, //   ( 49)  Q - 0x0051 Latin Capital Letter Q
	{ 0x7F,0x09,0x19,0x29,0x46}, //   ( 50)  R - 0x0052 Latin Capital Letter R
	{ 0x46,0x49,0x49,0x49,0x31}, //   ( 51)  S - 0x0053 Latin Capital Letter S
	{ 0x01,0x01,0x7F,0x01,0x01}, //   ( 52)  T - 0x0054 Latin Capital Letter T
	{ 0x3F,0x40,0x40,0x40,0x3F}, //   ( 53)  U - 0x0055 Latin Capital Letter U
	{ 0x1F,0x20,0x40,0x20,0x1F}, //   ( 54)  V - 0x0056 Latin Capital Letter V
	{ 0x3F,0x40,0x38,0x40,0x3F}, //   ( 55)  W - 0x0057 Latin Capital Letter W
	{ 0x63,0x14,0x08,0x14,0x63}, //   ( 56)  X - 0x0058 Latin Capital Letter X
	{ 0x07,0x08,0x70,0x08,0x07}, //   ( 57)  Y - 0x0059 Latin Capital Letter Y
	{ 0x61,0x51,0x49,0x45,0x43}, //   ( 58)  Z - 0x005A Latin Capital Letter Z
	{ 0x00,0x7F,0x41,0x41,0x00}, //   ( 59)  [ - 0x005B Left Square Bracket
	{ 0x02,0x04,0x08,0x10,0x20}, //   ( 60)  \ - 0x005C Reverse Solidus
	{ 0x00,0x41,0x41,0x7F,0x00}, //   ( 61)  ] - 0x005D Right Square Bracket
	{ 0x04,0x02,0x01,0x02,0x04}, //   ( 62)  ^ - 0x005E Circumflex Accent
	{ 0x40,0x40,0x40,0x40,0x40}, //   ( 63)  _ - 0x005F Low Line
	{ 0x01,0x02,0x04,0x00,0x00}, //   ( 64)  ` - 0x0060 Grave Accent
	{ 0x20,0x54,0x54,0x54,0x78}, //   ( 65)  a - 0x0061 Latin Small Letter A
	{ 0x7F,0x48,0x44,0x44,0x38}, //   ( 66)  b - 0x0062 Latin Small Letter B
	{ 0x38,0x44,0x44,0x44,0x20}, //   ( 67)  c - 0x0063 Latin Small Letter C
	{ 0x38,0x44,0x44,0x48,0x7F}, //   ( 68)  d - 0x0064 Latin Small Letter D
	{ 0x38,0x54,0x54,0x54,0x18}, //   ( 69)  e - 0x0065 Latin Small Letter E
	{ 0x08,0x7E,0x09,0x01,0x02}, //   ( 70)  f - 0x0066 Latin Small Letter F
	{ 0x06,0x49,0x49,0x49,0x3F}, //   ( 71)  g - 0x0067 Latin Small Letter G
	{ 0x7F,0x08,0x04,0x04,0x78}, //   ( 72)  h - 0x0068 Latin Small Letter H
	{ 0x00,0x44,0x7D,0x40,0x00}, //   ( 73)  i - 0x0069 Latin Small Letter I
	{ 0x20,0x40,0x44,0x3D,0x00}, //   ( 74)  j - 0x006A Latin Small Letter J
	{ 0x7F,0x10,0x28,0x44,0x00}, //   ( 75)  k - 0x006B Latin Small Letter K
	{ 0x00,0x41,0x7F,0x40,0x00}, //   ( 76)  l - 0x006C Latin Small Letter L
	{ 0x7C,0x04,0x18,0x04,0x7C}, //   ( 77)  m - 0x006D Latin Small Letter M
	{ 0x7C,0x08,0x04,0x04,0x78}, //   ( 78)  n - 0x006E Latin Small Letter N
	{ 0x38,0x44,0x44,0x44,0x38}, //   ( 79)  o - 0x006F Latin Small Letter O
	{ 0x7C,0x14,0x14,0x14,0x08}, //   ( 80)  p - 0x0070 Latin Small Letter P
	{ 0x08,0x14,0x14,0x18,0x7C}, //   ( 81)  q - 0x0071 Latin Small Letter Q
	{ 0x7C,0x08,0x04,0x04,0x08}, //   ( 82)  r - 0x0072 Latin Small Letter R
	{ 0x48,0x54,0x54,0x54,0x20}, //   ( 83)  s - 0x0073 Latin Small Letter S
	{ 0x04,0x3F,0x44,0x40,0x20}, //   ( 84)  t - 0x0074 Latin Small Letter T
	{ 0x3C,0x40,0x40,0x20,0x7C}, //   ( 85)  u - 0x0075 Latin Small Letter U
	{ 0x1C,0x20,0x40,0x20,0x1C}, //   ( 86)  v - 0x0076 Latin Small Letter V
	{ 0x3C,0x40,0x30,0x40,0x3C}, //   ( 87)  w - 0x0077 Latin Small Letter W
	{ 0x44,0x28,0x10,0x28,0x44}, //   ( 88)  x - 0x0078 Latin Small Letter X
	{ 0x0C,0x50,0x50,0x50,0x3C}, //   ( 89)  y - 0x0079 Latin Small Letter Y
	{ 0x44,0x64,0x54,0x4C,0x44}, //   ( 90)  z - 0x007A Latin Small Letter Z
};

// writing a byte to the LCD
void OLED_send_cmd(char val)
{
	i2c_repeated_start_OLED();
	i2c_write_val_to_reg_OLED(OLED_write_address,OLED_cmd_registry,val);
	i2c_stop();
}

void OLED_send_byte(char val)
{
	i2c_repeated_start_OLED();
	i2c_write_val_to_reg_OLED(OLED_write_address,OLED_val_registry,val);
	i2c_stop();
}

volatile void OLED_init()
{
	// as per LCD.cpp file from MiniWii
	// not clear about the start - repeated start options
	
	OLED_send_cmd(0xAE);	// display off
	OLED_send_cmd(0xA4);	// all pixels off
	OLED_send_cmd(0x20);	// set memory addressing mode (following message sets the value)
	OLED_send_cmd(0x02);	// set memory addressing mode to page addressing mode
	OLED_send_cmd(0xA1);	// column address 127 mapped to SEG0 (POR)
	OLED_send_cmd(0xC8);	// Scan from left to right
	OLED_send_cmd(0xA6);	// Set WHITE chars on BLACK background (Normal Display)
	OLED_send_cmd(0x81);	// Setup CONTRAST CONTROL, following byte is the contrast Value	(Contrast needs two commands after each other)
	OLED_send_cmd(0xAF);		// contrast value between 1 ( == dull) to 256 ( == bright)
	OLED_send_cmd(0xD6);	// Zoom
	OLED_send_cmd(0x01);	// Zoom on
	_delay_ms(100);
	OLED_send_cmd(0xAF);	// display on
}

volatile void OLED_clear()
{

	OLED_send_cmd(0xA6);	// Set WHITE chars on BLACK background (Normal Display)
	OLED_send_cmd(0xAE);	// display off
	OLED_send_cmd(0x20);	// set memory addressing mode
	OLED_send_cmd(0x00);	// set memory addressing mode to horizontal addressing mode
	OLED_send_cmd(0xB0);	// set page address to 0
	OLED_send_cmd(0x40);	// Display start line register to 0
	OLED_send_cmd(0x00);		// Set low col address to 0
	OLED_send_cmd(0x10);	// Set high col address to 0
	for (uint16_t i=0;i<1024;i++)
	{
		OLED_send_byte(0x00);	// Filling every pixel with blank
	}
	OLED_send_cmd(0x81);	// Setup CONTRAST CONTROL, following byte is the contrast Value
	OLED_send_cmd(0xAF);	// contrast value between 1 ( == dull) to 256 ( == bright) 
	OLED_send_cmd(0xAF);	// Turn Display On
	OLED_send_cmd(0xB0);	// set page address to 0
}

// Sending a number to the OLED
volatile void OLED_send_num(int32_t num)
{
	char k[31];
	ltoa(num, k, 10);	// convert long to ascii -> decimal system
	//OLED_send_string(k);
		if (k[0]=="0")
		{
			OLED_send_num_0();
		}
}

void OLED_send_char(char d)
{
	buffer = Font_Lookup[d-32][0];
	OLED_send_byte(buffer);
	buffer = Font_Lookup[d-32][1];
	OLED_send_byte(buffer);
	buffer = Font_Lookup[d-32][2];
	OLED_send_byte(buffer);
	buffer = Font_Lookup[d-32][3];
	OLED_send_byte(buffer);
	buffer = Font_Lookup[d-32][4];
	OLED_send_byte(buffer);
	OLED_send_byte(0);
	OLED_send_byte(0);
	buffer = 0;
}

void OLED_send_string(char *e)
{
	while(*e)
	{
		OLED_send_char(*e);
		*e++;
	}
	
}

volatile void OLED_set_position(uint8_t x, uint8_t y)
{
	// Lines only from 0 to 7
	//if (*line >= 8) *line = 7;
	//if (*line < 0) *line = 0;
	
	OLED_send_cmd(0xB0+(y));	// page address
	OLED_send_cmd(0x00+(8*(x)&0x0F));	     //set low col address	(Taking the y value (times 8 to achieve 1 CHAR shift) and Masking it for the First four bits)
	OLED_send_cmd(0x10+((8*(x)>>4)&0x0F));  //set high col address (Taking the y value (times 8 to achieve 1 CHAR shift), shifting it to position and Masking it for the First four bits)
}

volatile void OLED_set_pos_mid()
{
	OLED_send_cmd(0xB0);	// page address
	OLED_send_cmd(0x07);	     //set low col address	(Taking the y value and Masking it for the First four bits)	00 - 0F
	OLED_send_cmd(0x17);  //set high col address (Taking the y value, shifting it to position and Masking it for the First four bits) 10 - 1F
	
}

volatile void OLED_set_pos_0()
{
		// to reset the pointer to the upper left corner
		OLED_send_cmd(0xB0);	// set page address to 0 => Reset the pointer on the screen
		OLED_send_cmd(0x00);	// set col address low to 0
		OLED_send_cmd(0x10);	// set col address high to 0
}

void OLED_page(uint8_t page)
{
	switch(page)
	{
		case 0:
		// write "GPS" on first line
		//G is 71 as per ASCII Table
		OLED_send_char_L();
		OLED_send_char_I();
		OLED_send_char_G();
		OLED_send_char_H();
		OLED_send_char_T();

		// write "HOME" on second line
		OLED_set_position(0,1);
		OLED_send_char_H();
		OLED_send_char_O();
		OLED_send_char_M();
		OLED_send_char_E();

		// write "LON" on second line
		OLED_set_position(0,2);
		OLED_send_char_M();
		OLED_send_char_O();
		OLED_send_char_D();
		OLED_send_char_E();
		
		// write "HEAD" on second line
		OLED_set_position(0,3);
		OLED_send_char_H();
		OLED_send_char_E();
		OLED_send_char_A();
		OLED_send_char_D();
		
		OLED_set_pos_0();
		break;
		
		
		
	}
}

void OLED_send_char_A()
{
	OLED_send_byte(0x7E);
	OLED_send_byte(0x11);
	OLED_send_byte(0x11);
	OLED_send_byte(0x11);
	OLED_send_byte(0x7E);
	OLED_send_byte(0);
	OLED_send_byte(0);
}

void OLED_send_char_B()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x36); //   ( 34)  B - 0x0042 Latin Capital Letter B
	OLED_send_byte(0);
	OLED_send_byte(0);
}
void OLED_send_char_C()
{
	OLED_send_byte(0x3E);
	OLED_send_byte(0x41);
	OLED_send_byte(0x41);
	OLED_send_byte(0x41);
	OLED_send_byte(0x22); //   ( 35)  C - 0x0043 Latin Capital Letter C
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_D()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x41);
	OLED_send_byte(0x41);
	OLED_send_byte(0x22);
	OLED_send_byte(0x1C); //   ( 36)  D - 0x0044 Latin Capital Letter D
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_E()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x41); //   ( 37)  E - 0x0045 Latin Capital Letter E
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_F()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x09);
	OLED_send_byte(0x09);
	OLED_send_byte(0x09);
	OLED_send_byte(0x01); //   ( 38)  F - 0x0046 Latin Capital Letter F
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_G()
{
	OLED_send_byte(0x3E);
	OLED_send_byte(0x41);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x7A); //   ( 39)  G - 0x0047 Latin Capital Letter G
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_H()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0x7F); //   ( 40)  H - 0x0048 Latin Capital Letter H
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_I()
{
	OLED_send_byte(0x00);
	OLED_send_byte(0x41);
	OLED_send_byte(0x7F);
	OLED_send_byte(0x41);
	OLED_send_byte(0x00); //   ( 41)  I - 0x0049 Latin Capital Letter I
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_J()
{
	OLED_send_byte(0x20);
	OLED_send_byte(0x40);
	OLED_send_byte(0x41);
	OLED_send_byte(0x3F);
	OLED_send_byte(0x01); //   ( 42)  J - 0x004A Latin Capital Letter J
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_K()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x08);
	OLED_send_byte(0x14);
	OLED_send_byte(0x22);
	OLED_send_byte(0x41); //   ( 43)  K - 0x004B Latin Capital Letter K
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_L()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x40);
	OLED_send_byte(0x40);
	OLED_send_byte(0x40);
	OLED_send_byte(0x40); //   ( 44)  L - 0x004C Latin Capital Letter L
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_M()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x02);
	OLED_send_byte(0x0C);
	OLED_send_byte(0x02);
	OLED_send_byte(0x7F); //   ( 45)  M - 0x004D Latin Capital Letter M
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_N()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x04);
	OLED_send_byte(0x08);
	OLED_send_byte(0x10);
	OLED_send_byte(0x7F); //   ( 46)  N - 0x004E Latin Capital Letter N
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_O()
{
	OLED_send_byte(0x3E);
	OLED_send_byte(0x41);
	OLED_send_byte(0x41);
	OLED_send_byte(0x41);
	OLED_send_byte(0x3E); //   ( 47)  O - 0x004F Latin Capital Letter O
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_P()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x09);
	OLED_send_byte(0x09);
	OLED_send_byte(0x09);
	OLED_send_byte(0x06); //   ( 48)  P - OLED_send_byte(0x0050 Latin Capital Letter P
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_Q()
{
	OLED_send_byte(0x3E);
	OLED_send_byte(0x41);
	OLED_send_byte(0x51);
	OLED_send_byte(0x21);
	OLED_send_byte(0x5E); //   ( 49)  Q - 0x0051 Latin Capital Letter Q
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_R()
{
	OLED_send_byte(0x7F);
	OLED_send_byte(0x09);
	OLED_send_byte(0x19);
	OLED_send_byte(0x29);
	OLED_send_byte(0x46); //   ( 50)  R - 0x0052 Latin Capital Letter R
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_S()
{
	OLED_send_byte(0x46);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x49);
	OLED_send_byte(0x31); //   ( 51)  S - 0x0053 Latin Capital Letter S
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_T()
{
	OLED_send_byte(0x01);
	OLED_send_byte(0x01);
	OLED_send_byte(0x7F);
	OLED_send_byte(0x01);
	OLED_send_byte(0x01); //   ( 52)  T - 0x0054 Latin Capital Letter T
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_U()
{
	OLED_send_byte(0x3F);
	OLED_send_byte(0x40);
	OLED_send_byte(0x40);
	OLED_send_byte(0x40);
	OLED_send_byte(0x3F); //   ( 53)  U - 0x0055 Latin Capital Letter U
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_V()
{
	OLED_send_byte(0x1F);
	OLED_send_byte(0x20);
	OLED_send_byte(0x40);
	OLED_send_byte(0x20);
	OLED_send_byte(0x1F); //   ( 54)  V - 0x0056 Latin Capital Letter V
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_W()
{
	OLED_send_byte(0x3F);
	OLED_send_byte(0x40);
	OLED_send_byte(0x38);
	OLED_send_byte(0x40);
	OLED_send_byte(0x3F); //   ( 55)  W - 0x0057 Latin Capital Letter W
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_X()
{
	OLED_send_byte(0x63);
	OLED_send_byte(0x14);
	OLED_send_byte(0x08);
	OLED_send_byte(0x14);
	OLED_send_byte(0x63); //   ( 56)  X - 0x0058 Latin Capital Letter X
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_Y()
{
	OLED_send_byte(0x07);
	OLED_send_byte(0x08);
	OLED_send_byte(0x70);
	OLED_send_byte(0x08);
	OLED_send_byte(0x07); //   ( 57)  Y - 0x0059 Latin Capital Letter Y
		OLED_send_byte(0);
		OLED_send_byte(0);
}
void OLED_send_char_Z()
{
	OLED_send_byte(0x61);
	OLED_send_byte(0x51);
	OLED_send_byte(0x49);
	OLED_send_byte(0x45);
	OLED_send_byte(0x43); //   ( 58)  Z - 0x005A Latin Capital Letter Z
		OLED_send_byte(0);
		OLED_send_byte(0);
}

void OLED_send_char_SP()
{
	OLED_send_byte(0x00);
	OLED_send_byte(0x00);
	OLED_send_byte(0x00);
	OLED_send_byte(0x00);
	OLED_send_byte(0x00); //   SPACE
	OLED_send_byte(0);
	OLED_send_byte(0);
}

void OLED_send_num_0()
{
		OLED_send_byte(0x3E);
		OLED_send_byte(0x51);
		OLED_send_byte(0x49);
		OLED_send_byte(0x45);
		OLED_send_byte(0x3E); //   Number 0
		OLED_send_byte(0);
		OLED_send_byte(0);
}

void OLED_battery_100(uint8_t hor,uint8_t ver)
{
		OLED_set_pos_0();
		OLED_set_position(hor,ver);
		
		OLED_send_byte(0xF8);
		OLED_send_byte(0xF8);
		OLED_send_byte(0x08);
		OLED_send_byte(0xF8);
		OLED_send_byte(0xFC);
		
		OLED_send_byte(0xF4);
		OLED_send_byte(0xF4);
		OLED_send_byte(0xF4);
		OLED_send_byte(0xF4);
		OLED_send_byte(0xF4);
		
		OLED_send_byte(0xFC);
		OLED_send_byte(0xF8);
		OLED_send_byte(0x08);
		OLED_send_byte(0xF8);
		OLED_send_byte(0xF8);
		
		OLED_set_position(hor,ver+1);
		
		OLED_send_byte(0xFF);
		OLED_send_byte(0xFF);
		OLED_send_byte(0x80);
		OLED_send_byte(0xFF);	//80
		OLED_send_byte(0xFF);

		OLED_send_byte(0xFF);
		OLED_send_byte(0xFF);
		OLED_send_byte(0xFF);	// All lower 3 batterie bars are set
		OLED_send_byte(0xFF);
		OLED_send_byte(0xFF);
		
		OLED_send_byte(0xFF);	
		OLED_send_byte(0xFF);
		OLED_send_byte(0x80);
		OLED_send_byte(0xFF);
		OLED_send_byte(0xFF);
		
}

void OLED_battery_80(uint8_t hor,uint8_t ver)
{
	OLED_set_pos_0();
	OLED_set_position(hor,ver);
	
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	OLED_send_byte(0x08);
	OLED_send_byte(0xC8);
	OLED_send_byte(0xCC);
	
	OLED_send_byte(0xC4);
	OLED_send_byte(0xC4);
	OLED_send_byte(0xC4);
	OLED_send_byte(0xC4);
	OLED_send_byte(0xC4);
	
	OLED_send_byte(0xCC);
	OLED_send_byte(0xC8);
	OLED_send_byte(0x08);
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	
	OLED_set_position(hor,ver+1);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);	
	OLED_send_byte(0xFF);

	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
}

void OLED_battery_60(uint8_t hor,uint8_t ver)
{
	OLED_set_pos_0();
	OLED_set_position(hor,ver);
	
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0x0C);
	
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	
	OLED_send_byte(0x0C);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	
	OLED_set_position(hor,ver+1);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);

	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
}

void OLED_battery_40(uint8_t hor,uint8_t ver)
{
	OLED_set_pos_0();
	OLED_set_position(hor,ver);
	
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0x0C);
	
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	
	OLED_send_byte(0x0C);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	
	OLED_set_position(hor,ver+1);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFC);
	OLED_send_byte(0xFC);

	OLED_send_byte(0xFC);
	OLED_send_byte(0xFC);
	OLED_send_byte(0xFC);
	OLED_send_byte(0xFC);
	OLED_send_byte(0xFC);
	
	OLED_send_byte(0xFC);
	OLED_send_byte(0xFC);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
}

void OLED_battery_30(uint8_t hor,uint8_t ver)
{
	OLED_set_pos_0();
	OLED_set_position(hor,ver);
	
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0x0C);
	
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	
	OLED_send_byte(0x0C);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	
	OLED_set_position(hor,ver+1);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0xF0);
	OLED_send_byte(0xF0);

	OLED_send_byte(0xF0);
	OLED_send_byte(0xF0);
	OLED_send_byte(0xF0);
	OLED_send_byte(0xF0);
	OLED_send_byte(0xF0);
	
	OLED_send_byte(0xF0);
	OLED_send_byte(0xF0);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
}

void OLED_battery_0(uint8_t hor,uint8_t ver)
{
	OLED_set_pos_0();
	OLED_set_position(hor,ver);
	
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0x0C);
	
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	OLED_send_byte(0x04);
	
	OLED_send_byte(0x0C);
	OLED_send_byte(0x08);
	OLED_send_byte(0x08);
	OLED_send_byte(0xF8);
	OLED_send_byte(0xF8);
	
	OLED_set_position(hor,ver+1);
	
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);

	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	OLED_send_byte(0x80);
	OLED_send_byte(0xFF);
	OLED_send_byte(0xFF);
	
}










