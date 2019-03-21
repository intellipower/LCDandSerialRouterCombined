
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

#include "types.h"
#include "HD44780_display.h"
//#include "timera.h"
//#include "timerb.h"

#define		bitset(var,bitno) ((var) |= bitno)
#define		bitclr(var,bitno) ((var) &= ~(bitno))
#define     CR				0x0D
#define     LF				0x0A
#define		DISP_ON			0x0C	//LCD control constants
#define		DISP_OFF		0x08
#define		CLR_DISP		0x01
#define		CUR_HOME		0x02
#define		GOTO_LINE0		0x80
#define		GOTO_LINE1		0xC0
#define		GOTO_LINE2		0x94
#define		GOTO_LINE3		0xD4
#define		BUSY_FLAG_BIT	0x80

// Custom LCD characters
static uint8_t custom_chars[64] = {			 // ADDR, Comment
                                      0x1F,0x00,0x00,0x00,0x00,0x00,0x00,0x1F, // 0x00, Bar Empty
                                      0x1F,0x10,0x10,0x10,0x10,0x10,0x10,0x1F, // 0x01, Bar 1 vert
                                      0x1F,0x18,0x18,0x18,0x18,0x18,0x18,0x1F, // 0x02, Bar 2 vert
                                      0x1F,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1F, // 0x03, Bar 3 vert
                                      0x1F,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1F, // 0x04, Bar 4 vert
                                      0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F,0x1F, // 0x05, Bar 5 vert
                                      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // 0x06, Blank
                                      0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  // 0x07, Blank
                                  };

#define LCD_CMD_SIZE 200
int lcdDataTop = 0, lcdDataBottom = 0;
char lcdData[LCD_CMD_SIZE], lcdDataType[LCD_CMD_SIZE];

typedef enum {
	LCD_DATA,
	LCD_COMMAND,
	LCD_BUSY_WAIT,
	LCD_IDLE
} lcd_states_t;

lcd_states_t lcdState = LCD_IDLE;

int busyWaitSubState = 1;

// Function prototypes

void e_toggle(void);
void wait_for_lcd(void);
void write_data(uint8_t bytedata);
void write_cmd(uint8_t bytedata);
int lcdCmdPending(void);
void write_cmd_init(uint8_t bytedata);

// Toggle the E control line
void e_toggle(void){
	bitset(LCD_CTRL_PORT,E);
	bitclr(LCD_CTRL_PORT,E);
}

// Wait for busy flag to return low
void wait_for_lcd(void){
}

// Write data to the LCD
void write_data(uint8_t data){
	lcdDataTop++;
	if (lcdDataTop >= LCD_CMD_SIZE) {
		lcdDataTop = 0;
	}
	lcdDataType[lcdDataTop] = 'D';
	lcdData[lcdDataTop] = data;
}

// Write a command to the LCD
void write_cmd(uint8_t data){
	lcdDataTop++;
	if (lcdDataTop >= LCD_CMD_SIZE) {
		lcdDataTop = 0;
	}
	lcdDataType[lcdDataTop] = 'C';
	lcdData[lcdDataTop] = data;
}

int lcdCmdPending(void) {
	if (lcdDataTop != lcdDataBottom) {
		return TRUE;
	}
	else {
		return FALSE;
	}
}

void lcdWriteStateController(void) {
	switch (lcdState) {
	case LCD_DATA:
		LCD_ODATA_PORT = lcdData[lcdDataBottom];
		bitset(LCD_CTRL_PORT,RS);		// set data mode
		bitclr(LCD_CTRL_PORT,RW);		// set write
		e_toggle();
	
		bitclr(LCD_CTRL_PORT,RS);
		busyWaitSubState = 1;
		lcdState = LCD_BUSY_WAIT;
		break;
	case LCD_COMMAND:
		LCD_ODATA_PORT = lcdData[lcdDataBottom];
		bitclr(LCD_CTRL_PORT,RS);		//set LCD to CMD mode
		bitclr(LCD_CTRL_PORT,RW);
		e_toggle();
		busyWaitSubState = 1;
		lcdState = LCD_BUSY_WAIT;
		break;
	case LCD_BUSY_WAIT:
		switch (busyWaitSubState) {
		case 1:
			LCD_DIR_PORT = 0x00; 			// set data port to input
		
			bitclr(LCD_CTRL_PORT,RS);		// set LCD to CMD mode
			bitset(LCD_CTRL_PORT,RW);   	// set READ
			bitset(LCD_CTRL_PORT,E);		// toggle E
			busyWaitSubState++;				// go to next state
			break;
		case 2:
			// Check for busy flag, loop until clear
			bitclr(LCD_CTRL_PORT,E);		// toggle E
			bitset(LCD_CTRL_PORT,E);		// toggle E
			if (!(LCD_IDATA_PORT & BUSY_FLAG_BIT)) {	// if not busy...
				busyWaitSubState++;			// go to next state
			}
			break;
		case 3:
			bitclr(LCD_CTRL_PORT,E);		// toggle E
			bitclr(LCD_CTRL_PORT,RW);   	// set WRITE
			LCD_DIR_PORT = 0xFF; 			// set data port back to output
			busyWaitSubState++;				// go to next state
			break;
		default:
			lcdState = LCD_IDLE;
			break;
		}
		break;
	case LCD_IDLE:
		if (lcdCmdPending()) {
			lcdDataBottom++;
			if (lcdDataBottom >= LCD_CMD_SIZE) {
				lcdDataBottom = 0;
			}
			if (lcdDataType[lcdDataBottom] == 'D') {
				lcdState = LCD_DATA;
			}
			else {
				if (lcdDataType[lcdDataBottom] == 'C') {
					lcdState = LCD_COMMAND;
				}
			}
		}
		break;
	default:
		lcdState = LCD_IDLE;
		break;
	}
}

// The first time we write a command to the LCD we
// have to wait a long time between bytes.  This does.
void write_cmd_init(uint8_t idata){
	uint16_t i;

	for (i = 0; i < 60000; i++);
	LCD_ODATA_PORT = idata;
	bitclr(LCD_CTRL_PORT,RS);		//set LCD to CMD mode
	bitclr(LCD_CTRL_PORT,RW);
	e_toggle();
}

// Go to a specific location on the LCD
void LCD_goto_position(uint8_t row_num, uint8_t pos){
	uint8_t position;

	if (pos >= LCD_ROW_CHARS) return;
	if (row_num >= LCD_ROWS) return;

	switch (row_num){
	case 0: position = GOTO_LINE0 + pos;
		break;
	case 1: position = GOTO_LINE1 + pos;
		break;
	case 2: position = GOTO_LINE2 + pos;
		break;
	case 3: position = GOTO_LINE3 + pos;
		break;
	}
	write_cmd(position);
	return;
}

// Write a character to a specific location on the LCD
void LCD_putc_precise(uint8_t c, uint8_t row_num, uint8_t pos) {
	if (row_num >= LCD_ROWS + 1) return;
	if (pos >= LCD_ROW_CHARS + 1) return;

	LCD_goto_position(row_num, pos);
	write_data(c);
}

// Write a character to the current position
void LCD_putc(uint8_t c) {
	write_data(c);
}

// Write a buffer of data to a specific location
void LCD_putbuf(uint8_t *buf, uint8_t length, uint8_t row_num, uint8_t pos) {
	uint8_t i;

	if (pos + length >= LCD_ROW_CHARS + 1) return;
	if (row_num >= LCD_ROWS + 1) return;

	LCD_goto_position(row_num, pos);
	for (i = 0; i < length; i++) write_data(buf[i]);
}

// Write a string to the LCD at a specific location
void LCD_putstr(uint8_t *buf, uint8_t row_num, uint8_t pos) {
	uint8_t i;

	if (pos >= LCD_ROW_CHARS + 1) return;
	if (row_num >= LCD_ROWS + 1) return;

	LCD_goto_position(row_num, pos);
	i = 0;
	while(*buf) {
		write_data(*buf++);
		i++;
		if (pos + i >= LCD_ROW_CHARS + 1) return;
	}
}

// Clear the LCD
void LCD_clear(void) {
	write_cmd(CLR_DISP);
}

// Print a 16bit bcd number.
void print_bcd16_lcd(uint16_t num) {
	uint8_t i;

	for (i = 12; i > 0; i -= 4) {
		LCD_putc((uint8_t)((num >> i) & 0x000F) + '0');
	}
	LCD_putc((uint8_t)(num & 0x000F) + '0');
}

// Print a 8bit bcd number.
void print_bcd8_lcd(uint8_t num) {
	LCD_putc((uint8_t)((num >> 4) & 0x0F) + '0');
	LCD_putc((uint8_t)(num & 0x000F) + '0');
}

// Initialize the LCD
void LCD_init(void){
	uint8_t i;

	bitclr(LCD_CTRL_PORT,E);
	bitclr(LCD_CTRL_PORT,RS);
	bitclr(LCD_CTRL_PORT,RW);

	// Init LCD and turn on
	write_cmd_init(0x30);
	write_cmd_init(0x30);
	write_cmd_init(0x30);
	write_cmd_init(0x38);	// Write Command "Set Interface"
	write_cmd_init(0x08);	// Write Command "Enable Display/Cursor"
	write_cmd_init(0x01);	// Write Command "Clear and Home"
	write_cmd_init(0x06);	// Write Command "Set Cursor Move Direction"
	write_cmd_init(0x0C);	// Turn on display

	// Define custom characters
	write_cmd(0x40);
	for (i = 0; i < 64; i++) write_data(custom_chars[i]);
	write_cmd(0x80);
}



