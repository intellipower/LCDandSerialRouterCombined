#include  <msp430x54x.h>
#include "types.h"
#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file
#include "timerb.h"
#include "port1.h"
#include "utilities.h"

volatile struct timeT runTime;

volatile unsigned int timer_b_isr_count;

volatile unsigned int alarm1;
volatile unsigned int alarm1_status;
volatile unsigned int alarm1_en;

volatile unsigned int alarm2;
volatile unsigned int alarm2_status;
volatile unsigned int alarm2_en;

volatile unsigned int alarm3;
volatile unsigned int alarm3_status;
volatile unsigned int alarm3_en;

// getTime method to get snapshot of time data
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

void alarm1_set(volatile unsigned int value){
	__disable_interrupt();
	alarm1 = timer_b_isr_count + value;
	alarm1_en = TRUE;
	__enable_interrupt();
}

void alarm1_switch(volatile unsigned int enable){
	if (enable) alarm1_en = TRUE;
	else alarm1_en = FALSE;
}

unsigned int alarm1_read(void){
	if (alarm1_status) {
		alarm1_status = FALSE;
		return TRUE;
	}
	return FALSE;
}

void alarm2_set(volatile unsigned int value){
	__disable_interrupt();
	alarm2 = timer_b_isr_count + value;
	alarm2_en = TRUE;
	__enable_interrupt();
}

void alarm2_switch(volatile unsigned int enable){
	if (enable) alarm2_en = TRUE;
	else alarm2_en = FALSE;
}

unsigned int alarm2_read(void){
	if (alarm2_status) {
		alarm2_status = FALSE;
		return TRUE;
	}
	return FALSE;
}

void alarm3_set(volatile unsigned int value){
	__disable_interrupt();
	alarm3 = timer_b_isr_count + value;
	alarm3_en = TRUE;
	__enable_interrupt();
}

void alarm3_switch(volatile unsigned int enable){
	if (enable) alarm3_en = TRUE;
	else alarm3_en = FALSE;
}

unsigned int alarm3_read(void){
	if (alarm3_status) {
		alarm3_status = FALSE;
		return TRUE;
	}
	return FALSE;
}

void timerb_init(void){
	//TB0CCR0 = TIMERB_ISR_INTERVAL_1mSEC;
	#if X2_CLOCK_MHZ == 16		// Which frequency crystal is in X2
		TB0CCR0 = 16000;		// PWM Period, ACLK = 16e6/1000 ~1msec => 16000
	#else
		TB0CCR0 = 7373;			// PWM Period, ACLK = 7.3728e6/1000 ~1msec => 7373
	#endif
	//	TB0CTL = TBSSEL_2 + MC_2 + ID__8;	// SMCLK, contmode, /8
	TB0CTL = TBSSEL_2 + MC_2;			// SMCLK, contmode

	TB0CCTL0 = CCIE;					// CCR0 interrupt enabled

	timer_b_isr_count = 0;

	runTime.msec = 0;
	runTime.days = 0;

	/*
	alarm1 = 0;
	alarm1_status = FALSE;
	alarm1_en = FALSE;

	alarm2 = 0;
	alarm2_status = FALSE;
	alarm2_en = FALSE;

	alarm3 = 0;
	alarm3_status = FALSE;
	alarm3_en = FALSE;
	*/
}

// Timer B0 interrupt service routine
#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void){

	//static int Rs485timer = 0;				// used to turn off transmit of RS485 so it can receive
	
//	TBCCR0 += TIMERB_ISR_INTERVAL_1mSEC;	// Add Offset to CCR0
	#if X2_CLOCK_MHZ == 16		// Which frequency crystal is in X2
		TBCCR0 += 16000;		// PWM Period, ACLK = 16e6/1000 ~1msec => 16000
	#else
		TBCCR0 += 7373;		// PWM Period, ACLK = 7.3728e6/1000 ~1msec => 7373
	#endif

	timer_b_isr_count++;
	/*
	if (alarm1_en && (timer_b_isr_count == alarm1)){
		alarm1_status = TRUE;
		alarm1_en = FALSE;
	}
	if (alarm2_en && (timer_b_isr_count == alarm2)){
		alarm2_status = TRUE;
		alarm2_en = FALSE;
	}

	if (alarm3_en && (timer_b_isr_count == alarm3)){
		alarm3_status = TRUE;
		alarm3_en = FALSE;
	}
	*/

	runTime.msec++;
	if (runTime.msec >= (long) 86400000) {	// if we are at 24hours worth of msec
		runTime.days++;						// bump days
		runTime.msec = (long) 0;			// reset milliseconds
	}
	#if LCD_DISPLAY==TRUE
		button1_service(timer_b_isr_count);
		button2_service(timer_b_isr_count);
		button3_service(timer_b_isr_count);
		button4_service(timer_b_isr_count);
	#endif

	// turn off RS485 Transmit after delay to make sure last character is shifted out
	/*
	if (Rs485busy == TRUE) {				// sent from RS485 in uscia_UART.c
		Rs485busy = FALSE;					// reset flag
		Rs485timer = 10;					// set time delay
	}
	if (iRange(Rs485timer--, 0, -8)) {		// make sure we see range before we fall below it.
		if (Rs485busy == FALSE) {			// only turn off transmit when not transmitting
			P9OUT &= ~BIT6;					// turn off transmit enable
			Rs485timer = -10;				// only do this once
		}
	}
	if (Rs485timer <= -10) {
		Rs485timer = -10;					// don't roll over													// Turn on RS485 transmit enable
	}
	*/
	if((usart_tx_buffer_count(UPSNET_PORT)==0) && !(UCA2STAT & UCBUSY)) {
       	P9OUT &= ~BIT6;						// Turn off Transmit Enable to receive
    } else {
    	_NOP();								// break point
	}
}
