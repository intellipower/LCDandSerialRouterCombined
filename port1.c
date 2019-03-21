
#include "IAR_YES_NO.h"
#ifdef __IAR
	#include "io430.h"
#else
	#include <msp430x54x.h>
#endif

//#include "types.h"
#include "low_level_init.h"
#include "port1.h"
#include "timerb.h"

#define TRUE 0x01
#define FALSE 0x00

// Buttons do not trigger immediately when pressed.  They each have a
// delay during which they must be held to be active.  This is implemented
// using the timerb ISR counter.  Each button starts in the READY state.  When
// pressed, the next time timerb isr fires the button state gets set to WAITING
// the the delay associated with the button is added to the timerb isr counter
// value.  If the button is still held by the time the timerb isr counter
// reaches that value the button state goes to TRIGGERED.  When the main loop
// queries the button status, any buttons in the TRIGGERED state are flagged
// and returned to the READY state.

// Define the states for the button state machine
#define BUTTON_STATE_READY 0
#define BUTTON_STATE_WAITING 1
#define BUTTON_STATE_TRIGGERED 2

extern volatile int timer_a_isr_count;

// Flags whether a button has been pressed
volatile unsigned int button1_pressed;
volatile unsigned int button2_pressed;
volatile unsigned int button3_pressed;
volatile unsigned int button4_pressed;

// Button states machine variables
unsigned int button1_state;
unsigned int button2_state;
unsigned int button3_state;
unsigned int button4_state;

unsigned int button1_triggertime;
unsigned int button2_triggertime;
unsigned int button3_triggertime;
unsigned int button4_triggertime;

// Initialize the buttons
void buttons_init(void){
	button1_pressed = FALSE;
	button2_pressed = FALSE;
	button3_pressed = FALSE;
	button4_pressed = FALSE;

	button1_state = BUTTON_STATE_READY;
	button2_state = BUTTON_STATE_READY;
	button3_state = BUTTON_STATE_READY;
	button4_state = BUTTON_STATE_READY;

	P1DIR &= ~(BIT4+BIT5+BIT6+BIT7);  	// Set upper 4 to input for buttons
	//P1REN |= 0xF0;	// Enable resistors
	//P1OUT &= 0x0F;  	// Engage pull-down resistors
}

// Check to see if button 1 has been pressed
unsigned int button1_query(void) {
	__disable_interrupt();
	if (button1_pressed) {
		button1_pressed = FALSE;
		__enable_interrupt();
		return TRUE;
	}
	__enable_interrupt();
	return FALSE;
}

// Check to see if button 2 has been pressed
unsigned int button2_query(void) {
	__disable_interrupt();
	if (button2_pressed) {
		button2_pressed = FALSE;
		__enable_interrupt();
		return TRUE;
	}
	__enable_interrupt();
	return FALSE;
}

// Check to see if button 3 has been pressed
unsigned int button3_query(void) {
	__disable_interrupt();
	if (button3_pressed) {
		button3_pressed = FALSE;
		__enable_interrupt();
		return TRUE;
	}
	__enable_interrupt();
	return FALSE;
}

// Check to see if button 4 has been pressed
unsigned int button4_query(void) {
	__disable_interrupt();
	if (button4_pressed) {
		button4_pressed = FALSE;
		__enable_interrupt();
		return TRUE;
	}
	__enable_interrupt();
	return FALSE;
}

// This is the same as buttons_query but doesn't reset the buttonX_pressed value
unsigned int buttons_peek(void){
	unsigned int retval = 0;

	__disable_interrupt();

	if (button1_pressed) {
		retval |= BUTTON1;
	}
	if (button2_pressed) {
		retval |= BUTTON2;
	}
	if (button3_pressed) {
		retval |= BUTTON3;
	}
	if (button4_pressed) {
		retval |= BUTTON4;
	}

	__enable_interrupt();

	return retval;
}

// This is used by the main loop to query which buttons have been
// held long enough to trigger.
unsigned int buttons_query(void){
	unsigned int retval = 0;

	__disable_interrupt();

	if (button1_pressed) {
		retval |= BUTTON1;
		button1_pressed = FALSE;
	}
	if (button2_pressed) {
		retval |= BUTTON2;
		button2_pressed = FALSE;
	}
	if (button3_pressed) {
		retval |= BUTTON3;
		button3_pressed = FALSE;
	}
	if (button4_pressed) {
		retval |= BUTTON4;
		button4_pressed = FALSE;
	}

	__enable_interrupt();

	return retval;
}

// These routines implement the button state machines
void button1_service(unsigned int timer_b_isr_count){
	switch(button1_state){
	case BUTTON_STATE_READY:
		// If button is pressed, record the timerb isr counter value at which
		// it should trigger
		if (P1IN & BIT4) {
			button1_state = BUTTON_STATE_WAITING;
			button1_triggertime = timer_b_isr_count + BUTTON1_DURATION;
		}
		break;
	case BUTTON_STATE_WAITING:
		// if button is still pressed and has been pressed long enough,
		// trigger the button
		if (P1IN & BIT4) {
			if (timer_b_isr_count == button1_triggertime) {
				button1_state = BUTTON_STATE_TRIGGERED;
				button1_pressed = TRUE;
			}
		}
		// The button was not held long enough.  Reset state machine
		else button1_state = BUTTON_STATE_READY;
		break;
	case BUTTON_STATE_TRIGGERED:
		if ((P1IN & BIT4) == 0) button1_state = BUTTON_STATE_READY;
		break;
	default:
		button1_state = BUTTON_STATE_READY;
		button1_pressed = FALSE;
	}
}

void button2_service(unsigned int timer_b_isr_count){
	switch(button2_state){
	case BUTTON_STATE_READY:
		// If button is pressed, record the timerb isr counter value at which
		// it should trigger
		if (P1IN & BIT5) {
			button2_state = BUTTON_STATE_WAITING;
			button2_triggertime = timer_b_isr_count + BUTTON2_DURATION;
		}
		break;
	case BUTTON_STATE_WAITING:
		// if button is still pressed and has been pressed long enough,
		// trigger the button
		if (P1IN & BIT5) {
			if (timer_b_isr_count == button2_triggertime) {
				button2_state = BUTTON_STATE_TRIGGERED;
				button2_pressed = TRUE;
			}
		}
		// The button was not held long enough.  Reset state machine
		else button2_state = BUTTON_STATE_READY;
		break;
	case BUTTON_STATE_TRIGGERED:
		if ((P1IN & BIT5) == 0) button2_state = BUTTON_STATE_READY;
		break;
	default:
		button2_state = BUTTON_STATE_READY;
		button2_pressed = FALSE;
	}
}

void button3_service(unsigned int timer_b_isr_count){
	switch(button3_state){
	case BUTTON_STATE_READY:
		// If button is pressed, record the timerb isr counter value at which
		// it should trigger
		if (P1IN & BIT6) {
			button3_state = BUTTON_STATE_WAITING;
			button3_triggertime = timer_b_isr_count + BUTTON3_DURATION;
		}
		break;
	case BUTTON_STATE_WAITING:
		// if button is still pressed and has been pressed long enough,
		// trigger the button
		if (P1IN & BIT6) {
			if (timer_b_isr_count == button3_triggertime) {
				button3_state = BUTTON_STATE_TRIGGERED;
				button3_pressed = TRUE;
			}
		}
		// The button was not held long enough.  Reset state machine
		else button3_state = BUTTON_STATE_READY;
		break;
	case BUTTON_STATE_TRIGGERED:
		if ((P1IN & BIT6) == 0) button3_state = BUTTON_STATE_READY;
		break;
	default:
		button3_state = BUTTON_STATE_READY;
		button3_pressed = FALSE;
	}
}

void button4_service(unsigned int timer_b_isr_count){
	switch(button4_state){
	case BUTTON_STATE_READY:
		// If button is pressed, record the timerb isr counter value at which
		// it should trigger
		if (P1IN & BIT7) {
			button4_state = BUTTON_STATE_WAITING;
			button4_triggertime = timer_b_isr_count + BUTTON4_DURATION;
		}
		break;
	case BUTTON_STATE_WAITING:
		// if button is still pressed and has been pressed long enough,
		// trigger the button
		if (P1IN & BIT7) {
			if (timer_b_isr_count == button4_triggertime) {
				button4_state = BUTTON_STATE_TRIGGERED;
				button4_pressed = TRUE;
			}
		}
		// The button was not held long enough.  Reset state machine
		else button4_state = BUTTON_STATE_READY;
		break;
	case BUTTON_STATE_TRIGGERED:
		if ((P1IN & BIT7) == 0) button4_state = BUTTON_STATE_READY;
		break;
	default:
		button4_state = BUTTON_STATE_READY;
		button4_pressed = FALSE;
	}
}
