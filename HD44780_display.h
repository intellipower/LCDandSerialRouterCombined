#ifndef _HD44780_display_h_    // inclusion guard
#define _HD44780_display_h_

#include "types.h"

#define 	LCD_ROWS		4
#define		LCD_ROW_CHARS	20
#define     LCD_ODATA_PORT	P4OUT
#define     LCD_IDATA_PORT	P4IN
#define 	LCD_DIR_PORT	P4DIR
#define     LCD_CTRL_PORT	P3OUT
#define     E				0x04		//P3.2
#define		RW				0x02		//P3.1
#define     RS				0x01		//P3.0
//#define		E_CYCLE_TIME	_NOP();_NOP()		//Note: this define doesn't appear used...should have a terminating ";"?
#define     E_CYCLE_TIME  __NOP; __NOP;

// Reference any custom characters
#define BAR0_CHAR 0
#define BAR1_CHAR 1
#define BAR2_CHAR 2
#define BAR3_CHAR 3
#define BAR4_CHAR 4
#define BAR5_CHAR 5

void LCD_clear(void);
void LCD_goto_position(uint8_t row_num, uint8_t pos);
void LCD_putc_precise(uint8_t c, uint8_t row_num, uint8_t pos);
void LCD_putc(uint8_t c);
void LCD_putbuf(ubyte *buf, ubyte length, ubyte row_num, uint8_t pos);
void LCD_putstr(uint8_t *buf, uint8_t row_num, uint8_t pos);
void print_bcd16_lcd(uint16_t num);
void print_bcd8_lcd(uint8_t num);
void LCD_init(void);
void lcdWriteStateController(void);

#endif
