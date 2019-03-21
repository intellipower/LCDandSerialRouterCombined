
/*
 * FILE: uscia_UART.h
 * TARGET: MSP430x54x
 * SYSTEM:
 * AUTHOR: Dan Rhodes
 * DESCRIPTION:	 USART1 - device controller.
 */
 
#ifndef _drv_usart_h_	 // inclusion guard
#define _drv_usart_h_

#define CR 0x0D
#define LF 0x0A

//#define USART_BUFSIZE		256	 // default buffer size
#define USART_BUFSIZE		100	 // default buffer size

#define USART0_RX_INT_DISABLE		UCA0IE &= ~UCRXIE
#define USART0_RX_INT_ENABLE		UCA0IE |=  UCRXIE
#define USART0_TX_INT_DISABLE		UCA0IE &= ~UCTXIE
#define USART0_TX_INT_ENABLE		UCA0IE |=  UCTXIE
#define USART1_RX_INT_DISABLE		UCA1IE &= ~UCRXIE
#define USART1_RX_INT_ENABLE		UCA1IE |=  UCRXIE
#define USART1_TX_INT_DISABLE		UCA1IE &= ~UCTXIE
#define USART1_TX_INT_ENABLE		UCA1IE |=  UCTXIE
#define USART2_RX_INT_DISABLE		UCA2IE &= ~UCRXIE
#define USART2_RX_INT_ENABLE		UCA2IE |=  UCRXIE
#define USART2_TX_INT_DISABLE		UCA2IE &= ~UCTXIE
#define USART2_TX_INT_ENABLE		UCA2IE |=  UCTXIE
#define USART3_RX_INT_DISABLE		UCA3IE &= ~UCRXIE
#define USART3_RX_INT_ENABLE		UCA3IE |=  UCRXIE
#define USART3_TX_INT_DISABLE		UCA3IE &= ~UCTXIE
#define USART3_TX_INT_ENABLE		UCA3IE |=  UCTXIE
/*
char mspMsgTx[20];
char mspMsgRx[20];
*/
typedef enum {
	RS485,
	RS232
} portType_t;
/*												// Moved to system configuration files
typedef enum {
	UPS1_PORT,		// Port 0
	SNMP_PORT,		// Port 1
	UPSNET_PORT,	// Port 2 RS485
	USER_PORT = 3,	// Port 3 User and Upsilon
	UPS2_PORT = 3	// Port 3
} portName_t;
*/

struct uartDataStrucT {
	int bufferSize, txReadIndex, txWriteIndex, rxReadIndex, rxWriteIndex;
	int txCharCount, rxCharCount;
	portType_t portType;
	char txBuffer[USART_BUFSIZE];
	char rxBuffer[USART_BUFSIZE];
};

void usart_init(void);
void usart_tx_buffer_flush(int port);
char usart_putchar(volatile char cByte, int port);
int usart_rx_buffer_count(int port);
void usart_rx_buffer_flush(int port);
char usart_getchar(int port);
char usart_peekchar(int port);
void usart_putstr(volatile char *str, int port);
char usart_putbuffer(volatile char *str, int length, int port);
int usart_tx_buffer_count(int port);
// debug
#if defined RDAT07_DEBUG
void rdat07debug(void);
#endif
// debug end

#endif
