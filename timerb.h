#ifndef TIMERB_H_
#define TIMERB_H_

#include "system_config.h"						// includes file name for configuration
#include SYSTEM_CONFIG							// configuration header file

struct timeT {
	volatile int days;
	volatile long msec;
};

//volatile unsigned int timer_b_isr_count;

struct timeT getTime(void);
int timer(volatile struct timeT timestart, volatile long delay);

void timerb_init(void);

#endif /*TIMERB_H_*/
