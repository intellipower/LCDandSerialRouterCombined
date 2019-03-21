
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

//#include "types.h"
#include "timera.h"
#include "uscia_UART.h"
//#include "port1.h"
/*
#include "uscia2_UART.h"
#include "uscia3_UART.h"
*/

// Function Prototypes

__interrupt void Timer_A (void);    // Timer A interrupt routine

// End of Function Prototypes

/*
extern struct timeT {
	int days;
	long msec;
} runTime;
*/
#define TRUE 0x01
#define FALSE 0x00

//volatile struct timeT runTime;
volatile int timer_a_isr_count;

void timera_init(void){
	P1SEL |= BIT2;				// P1.2 = TA1
	P1DIR |= BIT2;				// set to output
	
	// This is the backlight control for the LCD.  PWM output is on P1.2
	
	//	TA0CCTL1 = OUTMOD_7;		// CCR1 reset/set
	//	TA0CCR1 = 20;				// CCR1 PWM duty cycle
	//	TA0CTL = TASSEL__ACLK + MC_1 + ID__8;	// ACLK, up mode, /8
	TA0CTL = TASSEL__ACLK + MC_2;	// ACLK, continuous mode
	//	TA0EX0 = TAIDEX_5;			// /6 - 1.5552e6/32768 = 47.461, 1.5552e6/(8*6)=32400
	//	TA0EX0 = TAIDEX_6;			// /7 - 1.5552e6/32768 = 47.461, 1.5552e6/(8*7)=27771.43
	//	TA0CCR0 = 1555;				// PWM Period, ACLK = 1.5552e6/1000 ~1msec => 1555
	//	TA0CCR0 = 1094;				// PWM Period, ACLK = 1.0944e6/1000 ~1msec => 1094
	/*
	#if X2_CLOCK_MHZ == 16		// Which frequency crystal is in X2
		TA0CCR0 = 16000;		// PWM Period, ACLK = 16e6/1000 ~1msec => 16000
	#else
		TA0CCR0 = 7373;			// PWM Period, ACLK = 7.3728e6/1000 ~1msec => 7373
	#endif
	TA0CCTL0 = CCIE;			// CCR0 Interrupt enabled
	*/
	// This is the backlight control for the LCD.  PWM output is on P1.2
	TA0CCR0 = 100;			// PWM Period, now using TA0CCR0 period for 1msec timer
	TA0CCTL1 = OUTMOD_7;		// CCR1 reset/set
	TA0CCR1 = 20;				// CCR1 PWM duty cycle
	
	//runTime.msec = 0;
	//runTime.days = 0;
}
	
// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void){
	// Not used for timer moved to timerb, use timera PWM for LCD backlight
	/*
	// Add Offset to CCR0
	#if X2_CLOCK_MHZ == 16		// Which frequency crystal is in X2
		TA0CCR0 += 16000;		// PWM Period, ACLK = 16e6/1000 ~1msec => 16000
	#else
		TA0CCR0 += 7373;		// PWM Period, ACLK = 7.3728e6/1000 ~1msec => 7373
	#endif
	runTime.msec++;
	if (runTime.msec >= (long) 86400000) {	// if we are at 24hours worth of msec
		runTime.days++;						// bump days
		runTime.msec = (long) 0;			// reset milliseconds
	}

	// turns off RS485 Transmit Enable when transmit buffer is empty and UART done with transmission
	if((usart_tx_buffer_count(UPSNET_PORT)==0) && !(UCA2STAT & UCBUSY)) {
       	P9OUT &= ~BIT6;						// Turn off Transmit Enable to receive
	}
	#if LCD_DISPLAY==TRUE
		button1_service();
		button2_service();
		button3_service();
		button4_service();
	#endif
	timer_a_isr_count++;
	*/
}

// getTime method to get snapshot of time data
/*
struct timeT getTime(void) {
	
	volatile struct timeT timeNow;

	__disable_interrupt();	// prevent corruption of data if interrupted
	timeNow = runTime;		//
	__enable_interrupt();	//
	
	return(timeNow);
}

// timer
// get start time and delay and return true if expired
int timer(volatile struct timeT timestart, volatile long delay) {
	volatile int expired;
	volatile struct timeT timeNow, timeAlarm;
	
	expired = FALSE;

	timeNow = getTime();

	timeAlarm = timestart;
	timeAlarm.msec += delay;
	 	
	if (timeAlarm.msec >= (long) 86400000) {	// if we are more than 24hours worth of msec
		timeAlarm.days++;					// bump days
		timeAlarm.msec -= (long) 86400000;		// remove a day's worth of msecs, leave overflow
	}
	
	// may make time delay more than a day, take that into account here
	if (timeNow.days > timeAlarm.days)  {
		expired = TRUE;
	}
	else {
		if ((timeNow.msec >= timeAlarm.msec) && (timeNow.days == timeAlarm.days)) {
			expired = TRUE;
		}
	}

	return(expired);
}
*/



