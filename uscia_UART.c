/*
 * FILE: uscia0_UART.c
 * TARGET: MSP430x54x
 * AUTHOR: Dan Rhodes
 * DESCRIPTION:	 Implements an interrupt driven, full duplex,
*				 buffered UART using uscia USART

	UPS1_PORT,		// Port 0
	SNMP_PORT,		// Port 1
	UPSNET_PORT,	// Port 2 RS485
	USER_PORT		// Port 3 User and Upsilon
*/

#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif
#include "uscia_UART.h"
#include "types.h"
#include "string.h"
#include "leds.h"
#include "main.h"

#define TRUE 0x01
#define FALSE 0x00

// #define USART_BUFFER_EMPTY 1	 // 1 = nothing to send

struct uartDataStrucT usartPort[4];			// Actual structure for each UART
extern char mspMsgTx[20];
extern char mspMsgRx[20];
int Rs485busy;

/*
Name: usart0_init
Description: init the RS232 driver system
Parameter: -
Returns: -
*/
void usart_init(void) {
	int port=0;

	for(port=0; port<=3; port++) {
		usartPort[port].txReadIndex = 0;
		usartPort[port].txWriteIndex = 0;
		usartPort[port].rxReadIndex = 0;
		usartPort[port].rxWriteIndex = 0;
		usartPort[port].bufferSize = USART_BUFSIZE;
		usartPort[port].txCharCount = 0;
		usartPort[port].rxCharCount = 0;
		usartPort[port].portType = RS232;
	}

	// UART0, UPS1
	UCA0CTL1 |= UCSSEL__SMCLK;	// SMCLK source
	#if BAUD_UPS1==9600
		UCA0BR0 = 0x82;			// 16 MHz 9600 baud = 0x682 = 1666.66
		UCA0BR1 = 0x06;			// upper byte
	#else
		#if BAUD_UPS1==4800
		UCA0BR0 = 0x05;			// 16 MHz 4800 baud = 0xd05 = 3333.3333 => 16MHz/3333 = 4800
		UCA0BR1 = 0x0d;			// upper byte
		#endif
	#endif

	P3SEL |= BIT4 + BIT5;	// P3.4,5 = USCI_A0 RXD/TXD
	P3DIR |= BIT4;			// P3.4 TXD
	P3DS  |= BIT4;			// drive high

	UCA0CTL1 &= ~UCSWRST;	// **Initialize USCI state machine**
	UCA0IE |= UCRXIE;		// Enable USCI_A0 RX interrupt
	
	// UART1, SNMP
	UCA1CTL1 |= UCSSEL__SMCLK;	// SMCLK source
	#if BAUD_SNMP==9600
		UCA1BR0 = 0x82;			// 16 MHz 9600 baud
		UCA1BR1 = 0x06;			// upper byte
	#else
		#if BAUD_SNMP==4800
		UCA1BR0 = 0x05;			// 16 MHz 4800 baud = 0xd05 = 3333.3333 => 16MHz/3333 = 4800
		UCA1BR1 = 0x0d;			// upper byte
		#endif
	#endif
	
	P5SEL |= BIT6+BIT7;		// P5.6,7 = USCI_A1 RXD/TXD and XT2
	P5DIR |= BIT6;
	P5DS  |= BIT6;
	
	UCA1CTL1 &= ~UCSWRST;	// **Initialize USCI state machine**
	UCA1IE |= UCRXIE;		// Enable USCI_A1 RX interrupt

	// UART2, UPSNET 
	UCA2CTL1 |= UCSSEL__SMCLK;	// SMCLK source
	UCA2CTL1 |= UCRXEIE;		// allow bad recieve to turn on interrupt flag

	#if BAUD_UPSNET==9600
		UCA2BR0 = 0x82;			// 16 MHz 9600 baud = 0x682 = 1666.66 
		UCA2BR1 = 0x06;			// upper byte
	#else
		#if BAUD_UPSNET==4800
		UCA2BR0 = 0x05;			// 16 MHz 4800 baud = 0xd05 = 3333.3333 => 16MHz/3333 = 4800
		UCA2BR1 = 0x0d;			// upper byte
		#endif
	#endif

	P9SEL |= BIT4+BIT5;			// P9.4,5 = USCI_A2 RXD/TXD and XT2
	P9DIR |= BIT4;
	P9DS  |= BIT4;
	
	P9DIR |= BIT6+BIT7;			// P9.6 is Transmit Enable, P9.7 is Terminating Resistor Enable
	P9DS  |= BIT6+BIT7;
	P9OUT |= BIT7;				// Turn on terminating resistor
	
	UCA2CTL1 &= ~UCSWRST;		// **Initialize USCI state machine**
	UCA2IE |= UCRXIE;			// Enable USCI_A1 RX interrupt
	Rs485busy = FALSE;

	// UART3, UPS2, User port and Upsilon
	UCA3CTL1 |= UCSSEL__SMCLK;	// SMCLK source

	#if BAUD_UPS2==9600
		UCA3BR0 = 0x82;			// 16 MHz 9600 baud = 0x682 = 1666.66
		UCA3BR1 = 0x06;			// upper byte
	#else
		#if BAUD_UPS2==4800
		UCA3BR0 = 0x05;			// 16 MHz 4800 baud = 0xd05 = 3333.3333 => 16MHz/3333 = 4800
		UCA3BR1 = 0x0d;			// upper byte
		#endif
	#endif

	P10SEL |= BIT4 + BIT5;	// P10.4,5 = USCI_A3 RXD/TXD and XT2
	P10DIR |= BIT4;				// TXD
	
	UCA3CTL1 &= ~UCSWRST;	// **Initialize USCI state machine**
	UCA3IE |= UCRXIE;			// Enable USCI_A3 RX interrupt
}

/*
Name: usart_putchar
Description: stores one char in TX buffer. If it's the first one,
send it immediately. Rest is sent by TXInterrupt automatically
Parameter: char cByte (to store in buffer)
Returns: -
*/
char usart_putchar(volatile char cByte, int port){
	
	if (usartPort[port].txCharCount >= usartPort[port].bufferSize) { // full up
		usart_tx_buffer_flush(port);		// reset buffers and counters (ie flush)
		return FALSE;					// indicate char not put in buffer
	}
	// load byte to buffer and inc index
	usartPort[port].txBuffer[usartPort[port].txWriteIndex++] = cByte;
	// if index past end of circular buffer, roll over
	if (usartPort[port].txWriteIndex >= usartPort[port].bufferSize) {
		usartPort[port].txWriteIndex = 0;
	}
	__disable_interrupt();
	usartPort[port].txCharCount++;		// new char, inc count
	switch(port) {
	case 0:
		USART0_TX_INT_ENABLE;			// Enable the transmit interrupt
		break;
	case 1:
		USART1_TX_INT_ENABLE;			// Enable the transmit interrupt
		break;
	case 2:
		USART2_TX_INT_ENABLE;			// Enable the transmit interrupt
		break;
	case 3:
		USART3_TX_INT_ENABLE;			// Enable the transmit interrupt
		break;
	default:
		break;
	}
	__enable_interrupt();

	return cByte;
}

/*
Name: usart0_putstr
Description: stores a string in the TX buffer. 
Parameter: ptr to string
Returns: -
*/
void usart_putstr(volatile char *str, int port){
	int length, i;

	length = strlen((char *)str);
	for (i = 0; i < length; i++) usart_putchar(str[i], port);

	#if defined COMM_MONITOR_PORT				// communication monitoring enabled
		switch (port) {							// process strings in Forever loop
		case UPS1_PORT:
			strcpy((char *) upsOne.StrCommMonitor, (char *) str);
			break;
		case UPS2_PORT:
			strcpy((char *) upsTwo.StrCommMonitor, (char *) str);
			break;
		}
	#endif										// defined COMM_MONITOR_PORT

//	while(*buffer) usart_putchar(*buffer++, port);
}

/*
Name: usart0_putbuffer
Description: stores a buffer in the TX buffer. 
Parameter: ptr to buffer, number of chars
Returns: -
*/
char usart_putbuffer(volatile char *str, int length, int port){
	int i;

	for (i = 0; i < length; i++) usart_putchar(str[i], port);

	return 0;
}

/*
Name: usart0_ISR
Description: uscia0 UART ISR
Parameter: -
Returns: -
*/
#pragma vector=USCI_A0_VECTOR
__interrupt void usart0_ISR(void){											// UPS1 port
	if (UCA0IFG & UCTXIFG){
		// This is a TX interrupt
		#if LCD_DISPLAY==FALSE
			RED3_ON;
			GRN3_OFF;
		#endif
		if (usartPort[0].txCharCount){										// send if chars are in buffer
			UCA0TXBUF = usartPort[0].txBuffer[usartPort[0].txReadIndex++]; // load tx register, inc index
			if (usartPort[0].txReadIndex >= usartPort[0].bufferSize) {	// end of circular buffer?
				usartPort[0].txReadIndex = 0;								// reset index
			}
			usartPort[0].txCharCount--;									// char sent, dec count
		}
		else {
			USART0_TX_INT_DISABLE;											// Turn off TX interrupt
		}
	}
	if (UCA0IFG & UCRXIFG){
		// This is an RX interrupt
		#if LCD_DISPLAY==FALSE
			GRN3_ON;
			RED3_OFF
		#endif
		usartPort[0].rxBuffer[usartPort[0].rxWriteIndex++] = UCA0RXBUF;	// store received byte and inc receive index
		if (usartPort[0].rxWriteIndex >= usartPort[0].bufferSize) {		// end of circular buffer?
			usartPort[0].rxWriteIndex = 0;									// reset index
		}
		usartPort[0].rxCharCount++;										// received, inc count
		//GRN0_OFF;
	}
}

#pragma vector=USCI_A1_VECTOR
__interrupt void usart1_ISR(void){											// SNMP Port
	if (UCA1IFG & UCTXIFG){
		// This is a TX interrupt
		#if LCD_DISPLAY==FALSE
			RED2_ON;
			GRN2_OFF;
		#endif
		if (usartPort[1].txCharCount){										// send if chars are in buffer
			UCA1TXBUF = usartPort[1].txBuffer[usartPort[1].txReadIndex++]; // load tx register, inc index
			if (usartPort[1].txReadIndex >= usartPort[1].bufferSize) {	// end of circular buffer?
				usartPort[1].txReadIndex = 0;								// reset index
			}
			usartPort[1].txCharCount--;									// char sent, dec count
		}
		else {
			USART1_TX_INT_DISABLE;											// Turn off TX interrupt
		}
	}
	if (UCA1IFG & UCRXIFG){
		// This is an RX interrupt
		#if LCD_DISPLAY==FALSE
			GRN2_ON;
			RED2_OFF;
		#endif
		usartPort[1].rxBuffer[usartPort[1].rxWriteIndex++] = UCA1RXBUF;	// store received byte and inc receive index
		if (usartPort[1].rxWriteIndex >= usartPort[1].bufferSize) {		// end of circular buffer?
			usartPort[1].rxWriteIndex = 0;									// reset index
		}
		usartPort[1].rxCharCount++;										// received, inc count
	}
}

#pragma vector=USCI_A2_VECTOR
__interrupt void usart2_ISR(void){											// UPSNET RS485 Port
	int i;
	if (UCA2IFG & UCTXIFG){
		// This is a TX interrupt
		#if LCD_DISPLAY==FALSE
			RED1_ON;
			GRN1_OFF;
		#endif
		if (usartPort[2].txCharCount){										// send if chars are in buffer
			P9OUT |= BIT6;													// Turn on RS485 transmit enable
			for(i=0;i<20;i++) {												// transmit enable propagation delay, so wait
				_NOP();
			}
			Rs485busy = TRUE;												// Tell msec timer char sent, it will turn off transmit after delay
			UCA2TXBUF = usartPort[2].txBuffer[usartPort[2].txReadIndex++]; // load tx register, inc index
			if (usartPort[2].txReadIndex >= usartPort[2].bufferSize) {
				usartPort[2].txReadIndex = 0;
			}
			usartPort[2].txCharCount--;									// char sent, dec count
		}
		else {
			USART2_TX_INT_DISABLE;											// Turn off TX interrupt
		}
	}
	if (UCA2IFG & UCRXIFG){
		// This is an RX interrupt
		#if LCD_DISPLAY==FALSE
			GRN1_ON;
			RED1_OFF;
		#endif
		if (usartPort[2].rxCharCount >= usartPort[2].bufferSize) return;
		usartPort[2].rxBuffer[usartPort[2].rxWriteIndex++] = UCA2RXBUF;	// store received byte and inc receive index
		if (usartPort[2].rxWriteIndex >= usartPort[2].bufferSize) {
			usartPort[2].rxWriteIndex = 0;
		}
		usartPort[2].rxCharCount++;										// received, inc count
	}
}

#pragma vector=USCI_A3_VECTOR
__interrupt void usart3_ISR(void){									// UPS2, User Port and Upsilon
	if (UCA3IFG & UCTXIFG){
		// This is a TX interrupt
		#if LCD_DISPLAY==FALSE
			RED0_ON;
			GRN0_OFF;
		#endif
		if (usartPort[3].txCharCount){								// send if chars are in buffer
			// load tx register, inc index
			UCA3TXBUF = usartPort[3].txBuffer[usartPort[3].txReadIndex++];
			// end of circular buffer?
			if (usartPort[3].txReadIndex >= usartPort[3].bufferSize) {
				usartPort[3].txReadIndex = 0;						// reset index
			}
			usartPort[3].txCharCount--;							// char sent, dec count
		}
		else {
			USART3_TX_INT_DISABLE;									// Turn off TX interrupt
		}
	}
	if (UCA3IFG & UCRXIFG){
		// This is an RX interrupt
		#if LCD_DISPLAY==FALSE
			GRN0_ON;
			RED0_OFF;
		#endif
		usartPort[3].rxBuffer[usartPort[3].rxWriteIndex++] = UCA3RXBUF;	// store received byte and inc receive index
		if (usartPort[3].rxWriteIndex >= usartPort[3].bufferSize) { // end of circular buffer?
			usartPort[3].rxWriteIndex = 0;			// reset index
		}
		usartPort[3].rxCharCount++;								// received, inc count
	}
}

/*
Name: usart0_rx_buffer_count
Description: How many chars are stored in RX buffer ?
if main routine wants to read chars, it has
to check first if RXBufCnt1() returns !=0
Parameter: -
Returns: number of chars in receive buffer
*/
int usart_rx_buffer_count(int port){
	return (usartPort[port].rxCharCount);
}

/*
Name: usart0_rx_buffer_flush
Description: Resets pointers and count to effectively "empty" buffer
Returns: nothing
*/
void usart_rx_buffer_flush(int port) {

	usartPort[port].rxReadIndex = 0;
	usartPort[port].rxWriteIndex = 0;
	usartPort[port].rxCharCount = 0;
}

/*
Name: getchar0
Description: Get one char from RX buffer. Multiple calls will
return all chars.
Parameter: -
Returns: next valid char in receive buffer
*/
char usart_getchar(int port){
	
	char cByte;
	
	if (usartPort[port].rxCharCount){						// char still available
		cByte = usartPort[port].rxBuffer[usartPort[port].rxReadIndex++];	// get byte from buffer
		if (usartPort[port].rxReadIndex >= usartPort[port].bufferSize) {
			usartPort[port].rxReadIndex = 0;
		}
		switch(port) {
		case 0:
			USART0_RX_INT_DISABLE;							// disable rx interrupt (IE2)
			break;
		case 1:
			USART1_RX_INT_DISABLE;							// disable rx interrupt (IE2)
			break;
		case 2:
			USART2_RX_INT_DISABLE;							// disable rx interrupt (IE2)
			break;
		case 3:
			USART3_RX_INT_DISABLE;							// disable rx interrupt (IE2)
			break;
		}
		//USART0_RX_INT_DISABLE;							// disable rx interrupt (IE2)
		usartPort[port].rxCharCount--;						// one char read, dec count
		//USART0_RX_INT_ENABLE;							// done, enable int (IE2)
		switch(port) {
		case 0:
			USART0_RX_INT_ENABLE;							// done, enable int (IE2)
			break;
		case 1:
			USART1_RX_INT_ENABLE;							// done, enable int (IE2)
			break;
		case 2:
			USART2_RX_INT_ENABLE;							// done, enable int (IE2)
			break;
		case 3:
			USART3_RX_INT_ENABLE;							// done, enable int (IE2)
			break;
		}
		return (cByte);
	}
	else return (0);
}

/*
Name: peekchar0
Description: non-destructively looks at next char from RX buffer.
Parameter: -
Returns: next valid char in receive buffer
*/
char usart_peekchar(int port){
	char cByte;
	if (usartPort[port].rxCharCount){										// char still available
		cByte = usartPort[port].rxBuffer[usartPort[port].rxReadIndex];	// get byte from buffer
		return (cByte);
	}
	else return (0);
}

/*
Name: usart0_tx_buffer_count
Description: How many chars are stored in TX buffer ?
Parameter: -
Returns: number of chars in TX buffer
*/
int usart_tx_buffer_count(int port){
	return (usartPort[port].txCharCount);
}

/*
Name: usart0_tx_buffer_flush
Description: Reset transmit pointers seeming to flush the buffer
Parameter: -
Returns: -
*/
void usart_tx_buffer_flush(int port) {

	usartPort[port].txReadIndex = 0;
	usartPort[port].txWriteIndex = 0;
	usartPort[port].txCharCount = 0;
}

int usartBufferCheck(void) {
	int port, bad;
	
	bad = FALSE;
	for(port=0;port<=3;port++) {
		if (usartPort[port].rxCharCount >= usartPort[port].bufferSize) { // full up
			usart_rx_buffer_flush(port);		// reset buffers and counters (ie flush)
			bad = TRUE;							// there was a problem
		}
	}
	return bad;									// if there was a problem
}

#if defined RDAT07_DEBUG
void rdat07debug(void) {
	// debug simulating answer to request to port 3
	static int i;
	static char StrTemp[50];
	
	strcpy((char *) StrTemp,"^RDAT07,0,1,1,2\r\n");
	i=0;
	while (StrTemp[i] != NULL) {
		usartPort[3].rxBuffer[usartPort[3].rxWriteIndex++] = StrTemp[i++];	// store received byte and inc receive index
		if (usartPort[3].rxWriteIndex >= usartPort[3].bufferSize) { // end of circular buffer?
			usartPort[3].rxWriteIndex = 0;			// reset index
		}
		usartPort[3].rxCharCount++;								// received, inc count
	}
	// debug end
}
#endif

