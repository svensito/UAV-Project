#ifndef UART_H
#define UART_H

#define UART_WORD_LENGTH		40
#define UART_COMPARE_SUCCESS	0
#define GPS_Params				13
#define GPS_chars				9

//Init Uart Function
void init_uart();

// write Functions that use conversion to ASCII
extern void write_char();
extern void write_string();
extern void write_string_ln();
extern void write_var(int16_t num);
extern void write_var_ln(int32_t num);
extern void write_ubyte_ln(uint8_t);
extern void write_ubyte(uint8_t);
// send functions that use unconverted values
extern void send_sbyte(int8_t);
extern void send_ubyte(uint8_t);
extern void send_sshort(int16_t);
extern void send_ushort(uint16_t);
extern void send_slong(int32_t);
extern void send_ulong(uint32_t);

int uart_reading_ascii;
volatile char uart_reading;
volatile char uart_word[UART_WORD_LENGTH];
volatile char uart_word_reset;
int uart_counter;

volatile char GPS_string[GPS_Params][GPS_chars];


#endif