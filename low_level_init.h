/*
 * FILE: low_level_init.h
 * TARGET: MSP430x54x
 * AUTHOR: Dan Rhodes
 * DESCRIPTION:  chip level initialization not done by compiler
*/

#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file

#ifndef _low_level_init_h_    // inclusion guard
#define _low_level_init_h_

#define FET_LED_OFF P1OUT &= 0xFE
#define FET_LED_ON  P1OUT |= 0x01

#define CUTTHROAT_MASTER_ON	P8OUT |= BIT0
#define CUTTHROAT_MASTER_OFF	P8OUT &= ~BIT0
#define CUTTHROAT_SLAVE_ON		P8OUT |= BIT1
#define CUTTHROAT_SLAVE_OFF	P8OUT &= ~BIT1

void low_level_init(void);    

#endif
