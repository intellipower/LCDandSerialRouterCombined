
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

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

// Function Prototypes

__interrupt void Timer_B (void);

// End of Funcion Prototypes

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
	long temptime, tempdays, timenow, daysnow;

	expired = FALSE;

	timeNow = getTime();

	timeAlarm = timestart;
//	timeAlarm.msec += delay;
	temptime = delay;
	timeAlarm.msec += temptime;

	if (timeAlarm.msec >= (long) 86400000) {	// if we are more than 24hours worth of msec
		timeAlarm.days++;					// bump days
		timeAlarm.msec -= (long) 86400000;		// remove a day's worth of msecs, leave overflow
	}

//	may make time delay more than a day, take that into account here
//	if (timeNow.days > timeAlarm.days)  {
	tempdays = timeAlarm.days;
	daysnow = timeNow.days;
	temptime = timeAlarm.msec;
	timenow = timeNow.msec;
	if (daysnow > tempdays)  
	{
		expired = TRUE;
	}
	else 
	{

//		if ((timeNow.msec >= timeAlarm.msec) && (timeNow.days == timeAlarm.days)) {
		if ((timenow >= temptime) && (daysnow == tempdays)) 
		{
			expired = TRUE;
		}
	}

	return(expired);
}


void timerb_init(void){

	TB0CCR0 = (unsigned int) (X2_CLOCK_FREQUENCY/1000);
	//	TB0CTL = TBSSEL_2 + MC_2 + ID__8;	// SMCLK, contmode, /8
	TB0CTL = TBSSEL_2 + MC_2;			// SMCLK, contmode

	TB0CCTL0 = CCIE;					// CCR0 interrupt enabled

	timer_b_isr_count = 0;

	runTime.msec = 0;
	runTime.days = 0;

}

// Timer B0 interrupt service routine
#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void){

	//static int Rs485timer = 0;				// used to turn off transmit of RS485 so it can receive
	
	TBCCR0 += (unsigned int) (X2_CLOCK_FREQUENCY/1000);		// PWM Period, ACLK = 16e6/1000 ~1msec => 16000
	timer_b_isr_count++;

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

	if((usart_tx_buffer_count(UPSNET_PORT)==0) && !(UCA2STAT & UCBUSY)) {
       	P9OUT &= ~BIT6;						// Turn off Transmit Enable to receive
    } else {
		__NOP;
	}
}
