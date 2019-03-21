/*
 * FILE: low_level_init.c
 * TARGET: MSP430x54x
 * AUTHOR: Dan Rhodes
 * DESCRIPTION:	 chip level initialization not done by compiler
*/
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
	#include "IAR_low_level_init.c"		// We need __low_level_init() function
#else						//   -- this runs before main() to turn of WDT
	#include <msp430x54x.h>
#endif

//#include "types.h"
#include "low_level_init.h"
#include "leds.h"
#include  "hal_pmm.h"
#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file

void low_level_init (void) {

	volatile long clockTimeout = 0;
	volatile int portMask;
	int tempMask;
	
	WDTCTL = WDTPW + WDTHOLD;		// Stop WDT == now done in __low_level_init()

	//	Program internal Low Voltage Drop Out core regulator to full voltage
	/*
	//	Step 1 - Program the SVM_L to the new level and wait for SVSMLDLYIFG to be set.
		SVSMLCTL = SVSMLRRL_3;			// Reset Release voltage = 1.84 volts
		while(!(PMMIFG & SVSMLDLYIFG)) 	// Wait for low side delay expired int flag
		PMMIFG &= ~SVSMLDLYIFG;			// reset flag
	//	Step 2 - Program PMMCOREV to the new Vcore level.
		PMMCTL0 = PMMCOREV_3;			// Set internal core voltage regulator to 1.85V for
										// full frequency, reset value = PMMCOREV_0 = 1.35V
	//	Step 3 - Wait for the voltage level reached (SVMLVLRIFG) interrupt.
		while(!(PMMIFG & SVMLVLRIFG)) 	// Wait for low side delay expired int flag
	//	Step 4 - Program the SVS_L to the new level
		SVSMLCTL = SVSLRVL_3;
	 */
	SetVCore(PMMCOREV_3);			// TODO - Core Voltage Regulator, 1.8V or 1.9V for full speed

	// Set unused pins to output
        
        portMask = BIT0+BIT1;
//	P1OUT &= ~(portMask);
        tempMask = portMask;       
        P1OUT &= ~tempMask;
	P1DIR = tempMask;
	
	portMask = BIT3+BIT6+BIT7;
//	P3OUT &= ~(portMask);
        tempMask = portMask;       
        P3OUT &= ~tempMask;
	P3DIR = tempMask;
	
	portMask = BIT1+BIT2+BIT3+BIT4+BIT5+BIT6+BIT7;
//      P6OUT &= ~(portMask);
        tempMask = portMask;       
        P6OUT &= ~tempMask;
	P6DIR = tempMask;
	
	portMask = BIT3+BIT4+BIT5+BIT6+BIT7;
//	P7OUT &= ~(portMask);
        tempMask = portMask;       
        P7OUT &= ~tempMask;
	P7DIR = tempMask;

	portMask = BIT2+BIT3+BIT4+BIT5+BIT6+BIT7;
//	P8OUT &= ~(portMask);
        tempMask = portMask;       
        P8OUT &= ~tempMask;
	P8DIR = tempMask;
	
	portMask = BIT0+BIT1+BIT2+BIT3;
//	P9OUT &= ~(portMask);
        tempMask = portMask;       
        P9OUT &= ~tempMask;
	P9DIR = tempMask;
	
	portMask = BIT6+BIT7;
//	P10OUT &= ~(portMask);
        tempMask = portMask;       
        P10OUT &= ~tempMask;
	P10DIR = tempMask;
	
	P2DIR = 0xFF;					// LEDs
	P2OUT = 0xFF;

	CUTTHROAT_MASTER_OFF;			// Make sure enabling output doesn't cutthroat the system
	CUTTHROAT_SLAVE_OFF;

	P8DIR = BIT0 + BIT1;			// Cutthroat Master & Slave
	P3DIR |= BIT0 + BIT1 + BIT2;	// P3.0,1,2 = LCD Ctrl lines
	P4DIR = 0xFF;					// LCD Data lines

	P5SEL = BIT2 + BIT3;			// P5.2,3 = XT2

	P7SEL |= 0x03;					// XT1

	//	UCSCTL6 &= ~(XT1OFF + XT2OFF);	// Set XT1 & XT2 On ***msj added
	//	UCSCTL6 &= ~(XT2OFF);			// Set only XT2 On ***msj added
	//	UCSCTL6 |= XCAP_3;				// Internal load cap X1 ***msj added

	P11DIR = BIT2 + BIT1 + BIT0;		// P11.2,1,0 to output direction
	#ifndef DEBUG_PORT					// If debug not using pins then...
		P11SEL = BIT2 + BIT1 + BIT0;	// Enable SMCLK, MCLK and ACLK outputs
	#endif

	// Set up the FLL
//	UCSCTL0 = ;							// DCO tap selection, MOD modulation bit counter
	UCSCTL1 = DCORSEL_5;				// DCORSEL selects DCO frequency range, DISMOD=1 disables FLL modulation
	UCSCTL2 = FLLD__1+488;				// FLLD FLL loop divider, FLLN FLL multiplier bits
	UCSCTL3	= SELREF__REFOCLK;			// SELREF selects clock input to FLL, FLLREFDIV input divide for input clock
//	UCSCTL4 = SELM__XT2CLK + SELS__XT2CLK + SELA__XT2CLK;	// Select XT2CLK for all clock sources
	UCSCTL4 = SELM__DCOCLK + SELS__DCOCLK + SELA__DCOCLK;	// Select XT2CLK for all clock sources
	//	UCSCTL5 = DIVA__16;				// divide ACLK input by 16, 24.8832e6/16 = 1.5552e6
//	USCTL6 = ;							// XT2DRIVE current drive select, XT2BYPASS external oscillator, XT2OFF
										// XT1DRIVE, XTS freq range select, XT1BYPASS, XCAP cap select, SMCLKOFF, XT1OFF
	UCSCTL6 &= ~(XT1OFF + XT2OFF);		// Set XT1 & XT2 On ***msj added
	UCSCTL6 |= XCAP_3;					// Internal load cap X1 ***msj added

	LED0_RED;							// Turn on top red led so it will be on if clock doesn't start

	// Loop until the crystals stabilize
	// Test oscillator fault flag and timeout, if flag clears or times out, continue
	while ( (SFRIFG1 & OFIFG) && (clockTimeout++ <= 1600000) ) {	
		//		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG); // clear
		SFRIFG1 &= ~OFIFG;				// Clear fault flags
	}
	if (clockTimeout < 1500000) {	// Oscillator test didn't time out, so use XT2 crystal oscillator
		// Select XT2CLK for all clock sources
		UCSCTL4 = SELM__XT2CLK + SELS__XT2CLK + SELA__XT2CLK;
		LED0_GRN;						// Clock started, turn green
	}
	else {
		LED0_YLW;								// Clock on DCO turn yellow
		clockTimeout = 0;						// reset clock
		while (clockTimeout++ <= 1600000);	// Keep yellow LED on for 1 second
	}

	FET_LED_ON;
}
